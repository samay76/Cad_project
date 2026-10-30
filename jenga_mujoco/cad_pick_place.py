#!/usr/bin/env python3
"""
cad_pick_place.py - Execute a grasp taught in the web CAD tool on the real CAD part in MuJoCo.

Launched by the web app's "Simulate in MuJoCo" button (vite-plugins/mujocoBridge.ts), or by hand:

  python cad_pick_place.py --step ../skf_cam_roller_305703_c-2z_0.stp --grasps grasps.json [--grasp-name "Grasp Pose 1"]
  python cad_pick_place.py --request runs/<id>/request.json          # what the web button does
  add --headless --snapshots out/ to run without a window and save PNGs per phase

Sequence: settle part on table -> preview ghost gripper at the taught grasp -> plan (IK + collision
checks for every waypoint, abort with a clear error if anything is unreachable/colliding) ->
approach -> grip -> lift -> inspection (part shown to a camera on the table in several positions,
images saved) -> transfer -> place in tray -> release -> retreat.

The approach is the path the user taught (see taught_path.py): either the straight pre-grasp ->
grasp line of a single saved pose, or the recorded motion (Record button), replayed relative to the
part with the recorded finger widths. Gripping closes the fingers ('outside' grip) or opens them
against a bore ('inside' grip); the retreat replays the taught path backwards.

Frames: see frames.py. The gripper pose relative to the part is exactly the exported T_part_tcp.
"""
import argparse
import json
import os
import sys
import time
import traceback

import numpy as np
import mujoco
import mujoco.viewer

from build_robot import build
from frames import (make_T, inv, T_to_pos_quat, web_pose_to_T, T_gbase_tcp, quat_xyzw_to_wxyz,
                    body_T, site_T, pose_error)
from ik import solve as ik_solve, HOME_QPOS, N_ARM
from step_to_mesh import convert
from cad_part import PART_BODY
from taught_path import from_grasp, densify, path_length
from cameras import add_inspection_camera, CAM_INSPECTION
from camera_feed import CameraFeed

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TCP_SITE = 'g_tcp'
GRIP_OPEN, GRIP_CLOSED = 0.0, 255.0
GRIPPER_NAMES = {'2f85': 'Robotiq 2F-85', 'hande': 'Robotiq Hand-E'}
GRIPPER_OPEN_MM = {'2f85': 85.0, 'hande': 50.0}
FINGER_JOINTS = ['g_left_driver_joint', 'g_left_spring_link_joint', 'g_left_follower',
                 'g_right_driver_joint', 'g_right_spring_link_joint', 'g_right_follower_joint']
GHOST_RGBA_OK = [0.1, 0.85, 0.35, 0.35]
GHOST_RGBA_BAD = [0.95, 0.15, 0.15, 0.45]


class PlanError(Exception):
    """Grasp cannot be executed (unreachable, colliding, ...). Message is shown to the user."""


# --------------------------------------------------------------------------------------------
# Status reporting (read by the web app through the Vite bridge)
# --------------------------------------------------------------------------------------------
class Status:
    def __init__(self, path):
        self.path = path
        self.info = {}

    def update(self, phase, message, state='running', ok=None):
        print(f'[{phase}] {message}', flush=True)
        if not self.path:
            return
        body = {'state': state, 'phase': phase, 'message': message, 'ok': ok, 'info': self.info,
                'time': time.time()}
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(body, f, indent=2)
        os.replace(tmp, self.path)


# --------------------------------------------------------------------------------------------
# Ghost gripper: a mocap copy of the gripper's visual meshes (open fingers) at the target pose
# --------------------------------------------------------------------------------------------
class FingerMap:
    """Finger opening (mm between the pad faces, as in the web tool) <-> actuator ctrl <-> joint angle.
    Measured once per run in a scratch simulation, so it always matches the loaded gripper model."""

    def __init__(self, model):
        d = mujoco.MjData(model)
        act = model.actuator('g_fingers_actuator').id
        jq = [model.jnt_qposadr[model.joint(n).id] for n in FINGER_JOINTS]
        gb = model.body('g_base').id
        pads = [model.geom(n).id for n in ('g_left_pad1', 'g_right_pad1')]
        half = model.geom_size[pads[0]][0]
        ctrl, gap, ang = [], [], []
        for c in np.linspace(0, 255, 11):
            mujoco.mj_resetData(model, d)
            d.qpos[:N_ARM] = d.ctrl[:N_ARM] = HOME_QPOS
            d.ctrl[act] = c
            for _ in range(int(1.0 / model.opt.timestep)):
                mujoco.mj_step(model, d)
            R = d.xmat[gb].reshape(3, 3)
            dl, dr = (R.T @ (d.geom_xpos[g] - d.xpos[gb]) for g in pads)
            ctrl.append(c)
            gap.append(max(0.0, (abs(dl[0] - dr[0]) - 2 * half) * 1000))
            ang.append(float(np.mean(d.qpos[jq])))
        # increasing gap for np.interp
        self.gap, self.ctrl, self.ang = np.array(gap[::-1]), np.array(ctrl[::-1]), np.array(ang[::-1])
        self.qadr = jq
        self.max_width = 85.0  # web tool's full opening for the 2F-85

    def ctrl_for(self, width_mm):
        return float(np.interp(width_mm, self.gap, self.ctrl))

    def angle_for(self, width_mm):
        return float(np.interp(width_mm, self.gap, self.ang))

    def set_qpos(self, qpos, width_mm):
        qpos[self.qadr] = self.angle_for(width_mm)


class HandeFingerMap:
    """Robotiq Hand-E (hande.py): parallel fingers, opening = 2 x joint travel, ctrl 0 = open .. 255 = closed."""

    def __init__(self, model):
        import hande
        self.max_width = hande.MAX_OPENING_MM
        self.qadr = [model.jnt_qposadr[model.joint(f'g_{n}').id] for n in hande.FINGER_JOINTS]

    def ctrl_for(self, width_mm):
        return float(255.0 * (1.0 - np.clip(width_mm / self.max_width, 0.0, 1.0)))

    def angle_for(self, width_mm):  # joint value (m) of each finger
        return float(np.clip(width_mm, 0.0, self.max_width) / 2000.0)

    def set_qpos(self, qpos, width_mm):
        qpos[self.qadr] = self.angle_for(width_mm)


def finger_map(model, gripper):
    return HandeFingerMap(model) if gripper == 'hande' else FingerMap(model)


def collect_ghost_geoms(model, fingers, width_mm):
    """Gripper visual meshes + pads, as (kind, meshname|size, T_gbase_geom), fingers at width_mm."""
    data = mujoco.MjData(model)
    data.qpos[:N_ARM] = HOME_QPOS
    fingers.set_qpos(data.qpos, width_mm)
    mujoco.mj_forward(model, data)
    T_wb_inv = inv(body_T(model, data, 'g_base'))
    out = []
    for g in range(model.ngeom):
        bname = model.body(model.geom_bodyid[g]).name
        if not bname.startswith('g_'):
            continue
        T_wg = np.eye(4)
        T_wg[:3, :3] = data.geom_xmat[g].reshape(3, 3)
        T_wg[:3, 3] = data.geom_xpos[g]
        T_bg = T_wb_inv @ T_wg
        gtype = model.geom_type[g]
        if gtype == mujoco.mjtGeom.mjGEOM_MESH and model.geom_contype[g] == 0:
            mid = model.geom_dataid[g]
            # compiled geom frame = authored frame * mesh offset -> undo the offset for re-authoring
            T_off = make_T(model.mesh_pos[mid], model.mesh_quat[mid])
            out.append(('mesh', model.mesh(mid).name, T_bg @ inv(T_off)))
        elif gtype == mujoco.mjtGeom.mjGEOM_BOX and 'pad' in model.geom(g).name:
            out.append(('box', model.geom_size[g].copy(), T_bg))
    return out


def add_ghost(spec, ghost_geoms, T_gb_tcp):
    body = spec.worldbody.add_body(name='ghost', mocap=True)
    for i, (kind, ref, T) in enumerate(ghost_geoms):
        p, q = T_to_pos_quat(T)
        kw = dict(name=f'ghost_{i}', pos=p, quat=q, contype=0, conaffinity=0, group=2, rgba=GHOST_RGBA_OK, mass=0)
        if kind == 'mesh':
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=ref, **kw)
        else:
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=ref, **kw)
    # TCP axes of the taught grasp (X red = closing axis, Y green, Z blue = approach)
    o = T_gb_tcp[:3, 3]
    for k, rgba in enumerate([[1, 0, 0, 0.9], [0, 1, 0, 0.9], [0, 0, 1, 0.9]]):
        tip = o + T_gb_tcp[:3, k] * 0.03
        body.add_geom(name=f'ghost_axis_{k}', type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=np.r_[o, tip],
                      size=[0.0015, 0, 0], contype=0, conaffinity=0, group=2, rgba=rgba, mass=0)


# --------------------------------------------------------------------------------------------
# Simulation runner
# --------------------------------------------------------------------------------------------
class CadPickPlace:
    def __init__(self, step_path, grasp, part_world_quat_wxyz, cfg, status, headless=False, snapshots=None,
                 feed_dir=None):
        self.cfg, self.status, self.grasp = cfg, status, grasp
        self.insp = dict(cfg.get('inspection') or {})
        self.feed_dir, self.feed = feed_dir, None
        self.headless, self.snap_dir = headless, snapshots
        self.viewer = self.renderer = None
        self.speed = float(cfg.get('speed', 1.0))

        status.update('converting', f'Converting {os.path.basename(step_path)} to meshes (cached after first run)...')
        self.meta = convert(step_path, density=cfg.get('density_kg_m3', 7850))
        status.info['part'] = {k: self.meta[k] for k in ('bbox_mm', 'mass_kg', 'volume_m3')}
        status.info['part']['hulls'] = len(self.meta['collision_hulls'])

        self.gripper = grasp.get('gripperId') or '2f85'
        if self.gripper not in GRIPPER_NAMES:
            raise PlanError(f"Unknown gripper '{self.gripper}' in the grasp (known: {', '.join(GRIPPER_NAMES)}).")
        status.info['gripper'] = GRIPPER_NAMES[self.gripper]
        self.tcp_offset_mm = float(grasp.get('tcpOffset', 145.0))
        self.pre_mm = float(grasp.get('preGraspDistance', 100.0))
        self.path = from_grasp(grasp, open_width_mm=GRIPPER_OPEN_MM[self.gripper])
        self.T_part_tcp = self.path.T_grasp
        status.info['taught_path'] = {'recorded': self.path.recorded, 'poses': len(self.path.Ts),
                                      'grip_mode': self.path.grip_mode,
                                      'pre_grip_width_mm': round(self.path.pre_grip_width, 1)}
        self.T_gb_tcp = T_gbase_tcp(self.tcp_offset_mm, self.gripper)

        # Place the part on the table in the same orientation it has in the web viewer.
        import trimesh
        verts = trimesh.load(os.path.join(self.meta['dir'], self.meta['visual_mesh']), force='mesh').vertices
        self.part_verts = np.asarray(verts)
        R = make_T([0, 0, 0], part_world_quat_wxyz)[:3, :3]
        rv = self.part_verts @ R.T
        c = (rv.min(0) + rv.max(0)) / 2
        px, py = cfg['pick_xy']
        part_pos = [px - c[0], py - c[1], -rv.min(0)[2] + 0.002]

        status.update('building', f'Building MuJoCo scene (CS66 + {GRIPPER_NAMES[self.gripper]} + part)...')
        build_kw = dict(obj='cad_part', cad_meta=self.meta, part_pos=part_pos, part_quat=part_world_quat_wxyz,
                        tcp_offset_mm=self.tcp_offset_mm, gravcomp=True, fine_fingers=True, gripper=self.gripper)
        model0 = build(**build_kw)
        self.fingers = finger_map(model0, self.gripper)
        ghost_geoms = collect_ghost_geoms(model0, self.fingers, self.path.pre_grip_width)
        def extra(s):
            add_ghost(s, ghost_geoms, self.T_gb_tcp)
            if self.insp.get('enabled'):
                add_inspection_camera(s, self.insp['camera_pos'], self.insp['point'], self.insp.get('fovy_deg', 45))

        self.model = build(**build_kw, extra=extra)
        self.data = mujoco.MjData(self.model)
        self.scratch = mujoco.MjData(self.model)

        m = self.model
        self.grip_act = m.actuator('g_fingers_actuator').id
        f = float(cfg.get('grip_force_limit_hande_N', 60.0) if self.gripper == 'hande'
                  else cfg.get('grip_force_limit', 5.0))
        m.actuator_forcerange[self.grip_act] = [-f, f]
        self.ghost_mocap = m.body_mocapid[m.body('ghost').id]
        self.ghost_geom_ids = [g for g in range(m.ngeom) if m.geom(g).name.startswith('ghost_')
                               and not m.geom(g).name.startswith('ghost_axis')]
        self.part_bid = m.body(PART_BODY).id
        self.robot_root = m.body('base_link').id
        self.ignore_env_bodies = {m.body('base_link').id, m.body('shoulder_link').id}
        self.pad_geoms = {g for g in range(m.ngeom) if m.geom(g).name.startswith(('g_left_pad', 'g_right_pad'))}

        self.data.qpos[:N_ARM] = HOME_QPOS
        self.data.ctrl[:N_ARM] = HOME_QPOS
        self.data.ctrl[self.grip_act] = GRIP_OPEN
        mujoco.mj_forward(m, self.data)
        self.path_points = []  # taught TCP path in world (drawn as dots)

    # ---------------- stepping / display ----------------
    def open_viewer(self):
        if self.feed_dir and self.insp.get('enabled'):
            self.feed = CameraFeed(self.model, CAM_INSPECTION, self.feed_dir)
            self.status.info['camera_feed'] = 'feed.jpg'
        if self.headless:
            if self.snap_dir:
                os.makedirs(self.snap_dir, exist_ok=True)
                self.renderer = mujoco.Renderer(self.model, 480, 640)
            return
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)
        self._aim(self.viewer.cam)

    def _draw_path(self, scn):
        """Taught TCP path as small dots (yellow = approach, green = grasp pose)."""
        pts = self.path_points
        for i, p in enumerate(pts):
            if scn.ngeom >= scn.maxgeom:
                break
            last = i == len(pts) - 1
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                [0.003 if last else 0.0018, 0, 0], p, np.eye(3).flatten(),
                                np.array([0.1, 0.9, 0.3, 0.9] if last else [1.0, 0.8, 0.1, 0.8], dtype=np.float32))
            scn.ngeom += 1

    def show_path(self, points):
        self.path_points = [np.asarray(p, dtype=float) for p in points]
        if self.viewer is not None:
            with self.viewer.lock():
                self.viewer.user_scn.ngeom = 0
                self._draw_path(self.viewer.user_scn)

    def _aim(self, cam):
        p = np.array(self.cfg['pick_xy']); q = np.array(self.cfg['place_xy'])
        cam.lookat[:] = [(p[0] + q[0]) / 2, (p[1] + q[1]) / 2, 0.08]
        cam.distance, cam.azimuth, cam.elevation = 0.95, 140, -28

    def overlay(self, title, detail=''):
        self._title, self._detail = title, detail
        if self.viewer is not None:
            self.viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                    title, detail)])

    def snapshot(self, name):
        if self.renderer is None:
            return
        cam = mujoco.MjvCamera()
        self._aim(cam)
        self.renderer.update_scene(self.data, cam)
        self._draw_path(self.renderer.scene)
        import PIL.Image
        PIL.Image.fromarray(self.renderer.render()).save(os.path.join(self.snap_dir, f'{name}.png'))

    def alive(self):
        return self.viewer is None or self.viewer.is_running()

    def step(self, seconds):
        m, d = self.model, self.data
        n = max(1, int(seconds / m.opt.timestep))
        t0 = time.time()
        for i in range(n):
            mujoco.mj_step(m, d)
            if self.feed is not None and i % 8 == 0:
                self.feed.push(d)
            if self.viewer is not None and i % 8 == 0:
                if not self.viewer.is_running():
                    raise KeyboardInterrupt
                self.viewer.sync()
                lag = (i + 1) * m.opt.timestep / self.speed - (time.time() - t0)
                if lag > 0:
                    time.sleep(lag)

    def move(self, q_waypoints, seconds, grip):
        """Smoothly drives the arm through joint waypoints (piecewise-linear, cosine time scaling).
        grip: gripper ctrl at the end, or one ctrl value per waypoint (recorded finger widths)."""
        m, d = self.model, self.data
        qs = [d.ctrl[:N_ARM].copy()] + [np.asarray(q) for q in q_waypoints]
        g0 = float(d.ctrl[self.grip_act])
        if np.ndim(grip) == 0:
            gs = [g0 + (grip - g0) * (i + 1) / len(q_waypoints) for i in range(len(q_waypoints))]
        else:
            gs = list(grip)
        gs = [g0] + gs
        n = max(2, int(seconds / self.speed / (m.opt.timestep * 10)))
        for i in range(n):
            s = 0.5 * (1 - np.cos(np.pi * (i + 1) / n))
            u = s * (len(qs) - 1)
            k = min(int(u), len(qs) - 2)
            a = u - k
            d.ctrl[:N_ARM] = (1 - a) * qs[k] + a * qs[k + 1]
            d.ctrl[self.grip_act] = (1 - a) * gs[k] + a * gs[k + 1]
            self.step(m.opt.timestep * 10 * self.speed)

    def path_seconds(self, Ts, minimum=1.5):
        lin, ang = path_length(Ts)
        return float(np.clip(lin / 0.06 + ang / 60.0, minimum, 20.0))

    # ---------------- planning ----------------
    def part_T(self, data=None):
        return body_T(self.model, data or self.data, PART_BODY)

    def part_min_z(self, T):
        return float((self.part_verts @ T[:3, :3].T + T[:3, 3])[:, 2].min())

    def ik(self, label, T, seeds):
        r = ik_solve(self.model, self.scratch, TCP_SITE, T, seeds)
        if not r.ok:
            p = T[:3, 3]
            raise PlanError(
                f"Grasp unreachable at '{label}': the closest arm pose misses the target by "
                f"{r.pos_err_mm:.1f} mm / {r.rot_err_deg:.1f} deg (TCP target x={p[0]:.3f} y={p[1]:.3f} "
                f"z={p[2]:.3f} m). Try a grasp whose approach points more downward, or move pick/place "
                f"positions in sim_config.json.")
        return r.q

    def follow(self, label, Ts, q_start, max_jump=0.6):
        """IK for each TCP pose of a (dense) path, seeded by the previous one. Rejects configuration flips."""
        qs, q_prev = [], q_start
        n = len(Ts)
        for i, T in enumerate(Ts):
            q = self.ik(f'{label} ({i + 1}/{n})', T, [q_prev])
            if np.max(np.abs(q - q_prev)) > max_jump:
                raise PlanError(f"'{label}' needs an arm reconfiguration mid-move at pose {i + 1}/{n} (joint jump "
                                f"{np.degrees(np.max(np.abs(q - q_prev))):.0f} deg); this motion is not feasible "
                                f"for the arm from here.")
            qs.append(q)
            q_prev = q
        return qs

    def line(self, label, T_a, T_b, q_start, n=8, max_jump=0.6):
        """IK along a straight TCP line (constant orientation)."""
        Ts = []
        for i in range(1, n + 1):
            T = T_a.copy()
            T[:3, 3] = T_a[:3, 3] + (T_b[:3, 3] - T_a[:3, 3]) * i / n
            Ts.append(T)
        return self.follow(label, Ts, q_start, max_jump)

    def check_collisions(self, label, q, check_part=True, width_mm=None):
        """Kinematic check with the fingers at width_mm (default: fully open) and the part at its current pose."""
        m, s = self.model, self.scratch
        s.qpos[:] = self.data.qpos
        s.qpos[:N_ARM] = q
        self.fingers.set_qpos(s.qpos, self.fingers.max_width if width_mm is None else width_mm)
        mujoco.mj_forward(m, s)
        tol = self.cfg.get('collision_tolerance_mm', 1.0) / 1000.0
        for c in s.contact[:s.ncon]:
            if c.dist > -tol:
                continue
            g1, g2 = c.geom1, c.geom2
            b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
            r1 = m.body_rootid[b1] == self.robot_root
            r2 = m.body_rootid[b2] == self.robot_root
            if r1 == r2:
                continue  # robot-robot or env-env
            rg, og = (g1, g2) if r1 else (g2, g1)
            rb, ob = m.geom_bodyid[rg], m.geom_bodyid[og]
            other = m.body(ob).name if ob != 0 else (m.geom(og).name or 'world')
            pen = -c.dist * 1000
            if ob == self.part_bid:
                if not check_part:
                    continue
                w = self.fingers.max_width if width_mm is None else width_mm
                if rg in self.pad_geoms:
                    hint = ('the part is wider than the gripper opening at this grasp.' if not self.path.recorded else
                            'the recorded motion runs the finger into the part in the simulation (web and sim '
                            'collision shapes differ by ~1 mm: keep a little clearance while recording).')
                    raise PlanError(f"At '{label}' finger pad {m.geom(rg).name} (fingers {w:.0f} mm open) "
                                    f"intersects the part by {pen:.1f} mm: {hint}")
                raise PlanError(f"Collision at '{label}': gripper/arm geom on '{m.body(rb).name}' (fingers "
                                f"{w:.0f} mm open) penetrates the part by {pen:.1f} mm. Move the grasp so only the "
                                f"finger pads touch the part.")
            if rb in self.ignore_env_bodies:
                continue
            raise PlanError(f"Collision at '{label}': '{m.body(rb).name}' hits '{other}' by {pen:.1f} mm "
                            f"(table/tray). Choose a grasp with more clearance from the table.")

    def plan(self):
        cfg, path = self.cfg, self.path
        T_wp = self.part_T()
        T_grasp = T_wp @ self.T_part_tcp
        lift = cfg['lift_height_m']

        def up(T, dz):
            T2 = T.copy()
            T2[2, 3] += dz
            return T2

        # Approach = taught path relative to the settled part, resampled every 4 mm / 4 deg.
        Ts_part, ws = densify(path.Ts, path.widths)
        Ts_w = [T_wp @ T for T in Ts_part]
        self.show_path([T[:3, 3] for T in Ts_w])

        # Place: same gripper orientation, part translated into the tray, bottom just above its floor.
        dz_place = cfg['place_floor_z'] + cfg['place_clearance_m'] - self.part_min_z(T_wp)
        # The part's COM (not its CAD origin) ends up over place_xy.
        com_w = T_wp[:3, :3] @ np.asarray(self.meta['com_m']) + T_wp[:3, 3]
        T_place = T_grasp.copy()
        T_place[:2, 3] += np.asarray(cfg['place_xy']) - com_w[:2]
        T_place[2, 3] += dz_place
        T_above_place = up(T_place, lift - dz_place)
        T_lift = up(T_grasp, lift)

        self.targets = dict(grasp=T_grasp, pre=Ts_w[0])
        start = 'start of recording' if path.recorded else 'pre-grasp'
        q_start = self.ik(start, Ts_w[0], [HOME_QPOS])
        q_approach = self.follow('taught approach' if path.recorded else 'approach', Ts_w[1:], q_start)
        q_lift = self.line('lift', T_grasp, T_lift, q_approach[-1])
        inspect, skipped = self.plan_inspection(T_wp, q_lift[-1], path.pre_grip_width)
        q_above = self.ik('above place', T_above_place, [q_lift[-1]])
        q_down = self.line('place', T_above_place, T_place, q_above)

        # Retreat = taught path backwards, relative to where the part is placed.
        T_wp_placed = T_place @ inv(T_grasp) @ T_wp
        rev_T = [T_wp_placed @ T for T in reversed(Ts_part[:-1])]
        rev_w = list(reversed(ws[:-1]))
        try:
            q_retreat = self.follow('retreat', rev_T, q_down[-1])
            for i in range(0, len(q_retreat), 3):
                self.check_collisions(f'retreat ({i + 1}/{len(q_retreat)})', q_retreat[i], check_part=False,
                                      width_mm=rev_w[i])
        except PlanError as e:  # e.g. the recording started where the tray wall is: back off straight instead
            print(f'Retreat along the taught path not possible ({e}); retreating straight back.', flush=True)
            back = make_T([0, 0, -self.pre_mm / 1000.0], [1, 0, 0, 0])
            q_retreat = self.line('retreat', T_place, T_place @ back, q_down[-1])
            rev_w = [path.pre_grip_width] * len(q_retreat)

        self.check_collisions(start, q_start, width_mm=ws[0])
        for i, q in enumerate(q_approach):
            try:
                self.check_collisions(f'{"taught approach" if path.recorded else "approach"} ({i + 1}/{len(q_approach)})',
                                      q, width_mm=ws[i + 1])
            except PlanError as e:
                hint = self.depth_hint(Ts_w[i + 1], ws[i + 1], q) if 'the part' in str(e) else ''
                raise PlanError(f'{e}{hint}') from None
        for i, q in enumerate(q_down[-3:]):
            self.check_collisions(f'place ({i + 1}/3)', q, check_part=False, width_mm=path.pre_grip_width)

        return dict(inspect=inspect, inspect_skipped=skipped,
                    start=q_start, approach=q_approach, approach_w=ws[1:], approach_T=Ts_w,
                    lift=q_lift, above=q_above, down=q_down, retreat=q_retreat, retreat_w=rev_w, retreat_T=rev_T)

    def plan_inspection(self, T_wp, q_seed, width_mm):
        """Arm motions that hold the part's COM at the inspection point and turn it through the configured
        orientations: yaw about the vertical, then tilt about the horizontal axis perpendicular to the
        camera's view (tilt > 0 turns the top towards the camera).
        Between views the part rotates in place about its COM (small IK steps), so it is not flung around
        by large joint-space swings. Views the arm cannot reach or that collide are skipped, not fatal.
        Returns ([(label, [q...]), ...], [skipped reasons])."""
        from scipy.spatial.transform import Rotation, Slerp
        ins = self.insp
        if not ins.get('enabled') or not ins.get('views'):
            return [], []
        P = np.asarray(ins['point'], dtype=float)
        view = P - np.asarray(ins['camera_pos'], dtype=float)
        vh = np.array([view[0], view[1], 0.0])
        vh /= np.linalg.norm(vh)
        tilt_axis = np.cross(vh, [0.0, 0.0, 1.0])
        com = np.asarray(self.meta['com_m'])

        def rot(axis, deg):
            return Rotation.from_rotvec(np.radians(deg) * np.asarray(axis, dtype=float)).as_matrix()

        def T_tcp_for(R, c):  # part orientation R with its COM at c -> TCP target
            T_part = np.eye(4)
            T_part[:3, :3] = R
            T_part[:3, 3] = c - R @ com
            return T_part @ self.T_part_tcp

        def motion(R_a, c_a, R_b, c_b, step_mm=10.0, step_deg=8.0):
            ang = np.degrees(Rotation.from_matrix(R_a.T @ R_b).magnitude())
            n = max(1, int(np.ceil(max(np.linalg.norm(c_b - c_a) * 1000 / step_mm, ang / step_deg))))
            sl = Slerp([0, 1], Rotation.from_matrix([R_a, R_b]))
            return [T_tcp_for(sl(i / n).as_matrix(), c_a + (c_b - c_a) * i / n) for i in range(1, n + 1)]

        R0 = T_wp[:3, :3]
        R_prev = R0
        c_prev = T_wp[:3, :3] @ com + T_wp[:3, 3] + [0, 0, self.cfg['lift_height_m']]  # COM after the lift
        poses, skipped, q_prev = [], [], q_seed
        for v in ins['views']:
            label = v.get('label', f"yaw {v.get('yaw_deg', 0)} tilt {v.get('tilt_deg', 0)}")
            R = rot(tilt_axis, v.get('tilt_deg', 0)) @ rot([0, 0, 1], v.get('yaw_deg', 0)) @ R0
            try:
                qs = self.follow(f'inspection {label}', motion(R_prev, c_prev, R, P), q_prev)
                for q in qs[::3] + [qs[-1]]:
                    self.check_collisions(f'inspection {label}', q, check_part=False, width_mm=width_mm)
            except PlanError as e:
                skipped.append(f'{label} ({str(e).split(":")[0]})')
                continue
            poses.append((label, qs))
            q_prev, R_prev, c_prev = qs[-1], R, P
        return poses, skipped

    def depth_hint(self, T, width_mm, q_seed, max_mm=40):
        """How far back along the TCP approach axis the colliding pose would be free (the finger is in too deep)."""
        for d in range(1, max_mm + 1):
            T_back = T @ make_T([0, 0, -d / 1000.0], [1, 0, 0, 0])
            r = ik_solve(self.model, self.scratch, TCP_SITE, T_back, [q_seed])
            if not r.ok:
                return ''
            try:
                self.check_collisions('depth search', r.q, width_mm=width_mm)
            except PlanError:
                continue
            return (f' The fingers go in too deep here: the finger/gripper gets wider further back from the '
                    f'gripping point (TCP), so only the fingertips fit. About {d} mm less deep is free: in the web tool stop the '
                    f'gripper ~{d + 1} mm earlier along the approach before opening/closing the fingers.')
        return ''

    # ---------------- run ----------------
    def set_ghost(self, T_world_tcp, ok=True):
        T_gb = T_world_tcp @ inv(self.T_gb_tcp)
        p, q = T_to_pos_quat(T_gb)
        self.data.mocap_pos[self.ghost_mocap] = p
        self.data.mocap_quat[self.ghost_mocap] = q
        for g in self.ghost_geom_ids:
            self.model.geom_rgba[g] = GHOST_RGBA_OK if ok else GHOST_RGBA_BAD
        mujoco.mj_forward(self.model, self.data)

    def hide_ghost(self):
        for g in range(self.model.ngeom):
            if self.model.geom(g).name.startswith('ghost_'):
                self.model.geom_rgba[g][3] = 0.0

    def phase(self, name, message):
        self.status.update(name, message)
        self.overlay(name.upper(), message)

    def run(self):
        st, cfg = self.status, self.cfg
        self.open_viewer()
        self.phase('settling', 'Letting the part settle on the table...')
        self.set_ghost(np.eye(4) @ make_T([0, 0, -5], [1, 0, 0, 0]))  # parked out of view while settling
        self.step(cfg['settle_seconds'])
        T_wp = self.part_T()
        drift = np.linalg.norm(T_wp[:2, 3] - np.asarray(self.cfg['pick_xy'])) * 1000
        st.info['part_settled_pose'] = {'pos_m': T_wp[:3, 3].round(4).tolist(),
                                        'quat_wxyz': T_to_pos_quat(T_wp)[1].round(4).tolist()}

        T_target = T_wp @ self.T_part_tcp
        self.set_ghost(T_target, ok=True)
        try:
            self.phase('checking', 'Checking reachability (IK) and collisions for every waypoint...')
            plan = self.plan()
        except PlanError as e:
            self.set_ghost(T_target, ok=False)
            self.overlay('GRASP NOT EXECUTABLE', str(e))
            self.snapshot('00_plan_failed')
            raise

        self.phase('preview', 'Green ghost = your taught grasp (TCP axes: X red, Y green, Z blue = approach).')
        self.snapshot('01_preview')
        self.step(cfg['preview_seconds'])

        path, fm = self.path, self.fingers
        if path.recorded:
            self.phase('start', 'Moving to where your recording starts...')
        else:
            self.phase('pre-grasp', f'Moving to pre-grasp ({self.pre_mm:.0f} mm back along the approach axis)...')
        self.move([plan['start']], 2.5, fm.ctrl_for(path.widths[0]))
        if path.recorded:
            self.phase('approach', f'Replaying your recorded motion ({len(plan["approach"])} poses)...')
        else:
            self.phase('approach', 'Approaching along the TCP +Z axis...')
        self.move(plan['approach'], self.path_seconds(plan['approach_T'], 2.0),
                  [fm.ctrl_for(w) for w in plan['approach_w']])
        self.step(0.4)

        # Frame check: actual TCP relative to the actual part vs the taught grasp.
        T_part_tcp_sim = inv(self.part_T()) @ site_T(self.model, self.data, TCP_SITE)
        e_mm, e_deg = pose_error(T_part_tcp_sim, self.T_part_tcp)
        st.info['grasp_pose_error'] = {'mm': round(e_mm, 2), 'deg': round(e_deg, 2)}
        self.snapshot('02_at_grasp')
        self.hide_ghost()

        inside = path.grip_mode == 'inside'
        grip_ctrl = GRIP_OPEN if inside else GRIP_CLOSED  # drive until the force limit stops the fingers
        self.phase('grip', (f'Opening the fingers against the inside of the part' if inside else 'Closing the gripper')
                   + f' (TCP vs taught grasp: {e_mm:.1f} mm / {e_deg:.1f} deg)...')
        self.move([plan['approach'][-1]], 1.0, grip_ctrl)
        self.step(0.6)
        T_tcp_part0 = inv(site_T(self.model, self.data, TCP_SITE)) @ self.part_T()
        z0 = self.part_T()[2, 3]

        self.phase('lift', 'Lifting the part...')
        self.move(plan['lift'], 2.0, grip_ctrl)
        self.step(0.3)
        self.snapshot('03_lifted')
        lifted = self.part_T()[2, 3] - z0
        if lifted < 0.5 * cfg['lift_height_m']:
            raise PlanError(f'The part was not picked up (rose {lifted * 1000:.0f} mm of '
                            f"{cfg['lift_height_m'] * 1000:.0f} mm). The grasp does not hold it: check that both "
                            f'finger pads touch the part, or raise grip_force_limit in sim_config.json.')

        if plan['inspect_skipped']:
            st.info['inspection_skipped'] = plan['inspect_skipped']
        if plan['inspect']:
            ins, n = self.insp, len(plan['inspect'])
            st.info['inspection'] = []
            for i, (label, qs) in enumerate(plan['inspect']):
                self.phase(f'inspect {i + 1}/{n}', f'Inspection {i + 1}/{n}: showing the part to the table camera ({label})...')
                self.move(qs, ins.get('move_seconds', 2.0) * (1.0 if i else 1.5), grip_ctrl)
                self.step(ins.get('hold_seconds', 0.8))
                fname = f'inspect_{i + 1:02d}.png'
                if self.feed is not None:
                    self.feed.snap(self.data, fname)
                slip_v, _ = pose_error(inv(site_T(self.model, self.data, TCP_SITE)) @ self.part_T(), T_tcp_part0)
                st.info['inspection'].append({'label': label, 'file': fname, 'slip_mm': round(slip_v, 1)})
                self.snapshot(f'03b_inspect_{i + 1:02d}')
            if self.part_T()[2, 3] - z0 < 0.5 * cfg['lift_height_m']:
                raise PlanError('The part slipped out of the gripper during the inspection moves. Try a firmer grasp '
                                '(more finger contact) or fewer/smaller inspection tilts in sim_config.json.')

        self.phase('transfer', 'Carrying the part to the tray...')
        self.move([plan['above']], 3.0, grip_ctrl)
        slip_mm, slip_deg = pose_error(inv(site_T(self.model, self.data, TCP_SITE)) @ self.part_T(), T_tcp_part0)
        st.info['slip_during_transport'] = {'mm': round(slip_mm, 2), 'deg': round(slip_deg, 2)}

        self.phase('place', 'Lowering into the tray...')
        self.move(plan['down'], 2.0, grip_ctrl)
        self.phase('release', f'Releasing: fingers back to {path.pre_grip_width:.0f} mm (as before the grip)...')
        self.move([plan['down'][-1]], 1.0, fm.ctrl_for(path.pre_grip_width))
        self.step(0.4)
        self.phase('retreat', 'Retreating along your taught path in reverse...' if path.recorded else 'Retreating...')
        self.move(plan['retreat'], self.path_seconds(plan['retreat_T'], 1.5),
                  [fm.ctrl_for(w) for w in plan['retreat_w']])
        self.move([HOME_QPOS], 2.5, GRIP_OPEN)
        self.step(0.8)
        self.snapshot('04_done')

        T_end = self.part_T()
        px, py = cfg['place_xy']
        in_tray = abs(T_end[0, 3] - px) < 0.065 and abs(T_end[1, 3] - py) < 0.05 and self.part_min_z(T_end) < 0.03
        st.info['part_in_tray'] = bool(in_tray)
        summary = (f"{'Part placed in the tray' if in_tray else 'Part was NOT placed in the tray'}. "
                   f"TCP vs taught grasp: {e_mm:.1f} mm / {e_deg:.1f} deg. "
                   f"Slip during transport: {slip_mm:.1f} mm / {slip_deg:.1f} deg."
                   + (f' (Part rolled {drift:.0f} mm while settling.)' if drift > 10 else ''))
        if plan['inspect']:
            summary += f" Inspection: {len(plan['inspect'])} camera images."
        if plan['inspect_skipped']:
            summary += f" Skipped views: {'; '.join(plan['inspect_skipped'])}."
        return in_tray, summary

    def idle_until_closed(self):
        if self.viewer is None:
            return
        try:
            while self.viewer.is_running():
                self.step(0.05)
        except KeyboardInterrupt:  # window closed by the user: normal end
            pass


# --------------------------------------------------------------------------------------------
def load_inputs(args):
    if args.request:
        with open(args.request) as f:
            req = json.load(f)
        grasp, step = req['grasp'], req['partFile']
        pwp = req.get('partWorldPose')
    else:
        if not (args.step and args.grasps):
            raise SystemExit('Give --request, or --step and --grasps')
        with open(args.grasps) as f:
            data = json.load(f)
        grasps = data if isinstance(data, list) else [data]
        if args.grasp_name:
            match = [g for g in grasps if g.get('name') == args.grasp_name]
            if not match:
                raise SystemExit(f"No grasp named '{args.grasp_name}'. Available: {[g.get('name') for g in grasps]}")
            grasp = match[0]
        else:
            grasp = grasps[args.grasp_index]
        step, pwp = args.step, None
    if not grasp.get('tcpPose'):
        raise SystemExit('Grasp has no tcpPose')
    q = quat_xyzw_to_wxyz([pwp['orientation'][k] for k in 'xyzw']) if pwp else np.array([1.0, 0, 0, 0])
    return step, grasp, q


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--request', help='request.json written by the web app bridge')
    ap.add_argument('--status', help='status.json to update (polled by the web app)')
    ap.add_argument('--step', help='STEP file (manual mode)')
    ap.add_argument('--grasps', help='grasps JSON exported from the web app (manual mode)')
    ap.add_argument('--grasp-name')
    ap.add_argument('--grasp-index', type=int, default=0)
    ap.add_argument('--config', default=os.path.join(BASE_DIR, 'sim_config.json'))
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--snapshots', help='directory for per-phase PNGs (headless)')
    ap.add_argument('--feed-dir', help='where the inspection camera feed/images go (default: the --status folder)')
    args = ap.parse_args()

    status = Status(args.status)
    sim = None
    try:
        with open(args.config) as f:
            cfg = json.load(f)
        step, grasp, q_part = load_inputs(args)
        status.info['grasp'] = grasp.get('name')
        feed_dir = args.feed_dir or (os.path.dirname(os.path.abspath(args.status)) if args.status else None)
        sim = CadPickPlace(step, grasp, q_part, cfg, status, args.headless, args.snapshots, feed_dir)
        ok, summary = sim.run()
        status.update('done', summary, state='done', ok=ok)
        sim.overlay('DONE' if ok else 'FINISHED (NOT IN TRAY)', summary + '  Close this window when finished.')
        sim.idle_until_closed()
        return 0 if ok else 2
    except PlanError as e:
        status.update('failed', str(e), state='failed', ok=False)
        if sim is not None:
            sim.idle_until_closed()
        return 3
    except KeyboardInterrupt:
        status.update('closed', 'MuJoCo window closed before the sequence finished.', state='failed', ok=False)
        return 4
    except Exception as e:  # noqa: BLE001
        traceback.print_exc()
        status.update('failed', f'Simulator error: {type(e).__name__}: {e}', state='failed', ok=False)
        return 1
    finally:
        if sim is not None and sim.feed is not None:
            sim.feed.close()


if __name__ == '__main__':
    sys.exit(main())
