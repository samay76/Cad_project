#!/usr/bin/env python3
"""
cad_pick_place.py - The MuJoCo cell simulator shared by the web app's jobs (auto_pick_place.py, real_robot.py).

CadPickPlace builds the scene for one CAD part: the EC66 with the Robotiq 2F-85 (and the calibrated base offset and
coupling plate), the table, the tray, the pick fixture with the part seated on it (pick_station.py), and a "ghost"
gripper that shows the target grasp. It converts the part's STEP once (step_to_mesh.py, cached), and provides the
stepping, smooth joint moves, viewer/snapshot and status reporting that AutoPickPlace uses to plan and run a pick.

Frames: see frames.py.
"""
import json
import os
import time

import numpy as np
import mujoco
import mujoco.viewer

from build_robot import build
from frames import make_T, inv, T_to_pos_quat, web_pose_to_T, T_gbase_tcp, body_T, site_T
from ik import solve as ik_solve, N_ARM
from robots import get_robot
from step_to_mesh import convert, is_mesh_file
from cad_part import PART_BODY
from pick_station import from_config as from_station_config, is_seated_part, seated_pose

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TCP_SITE = 'g_tcp'
GRIP_OPEN, GRIP_CLOSED = 0.0, 255.0
GRIPPER_NAMES = {'2f85': 'Robotiq 2F-85'}
GRIPPER_OPEN_MM = {'2f85': 85.0}
# How the arm goes to pick (sim_config.json "pick_motion" overrides these). Speeds are peak values.
PICK_MOTION_DEFAULTS = {
    'home_tcp_m': [0.40, 0.0, 0.40],  # home: gripper pointing straight down, up over the middle of the table
    'orient_first': True,             # turn the gripper at home, then travel without turning
    'orient_speed_rad_s': 0.5,        # joint speed while the gripper turns at home
    'travel_speed_m_s': 0.12,         # TCP speed from home to the pre-grasp
    'approach_speed_m_s': 0.03,       # TCP speed of the final approach onto the part
    'grip_seconds': 1.5,              # closing (or opening, inside grips) the fingers
    'lift_speed_m_s': 0.04,           # TCP speed lifting the part off the table
}
FINGER_JOINTS = ['g_left_driver_joint', 'g_left_spring_link_joint', 'g_left_follower',
                 'g_right_driver_joint', 'g_right_spring_link_joint', 'g_right_follower_joint']
GHOST_RGBA_OK = [0.1, 0.85, 0.35, 0.35]
GHOST_RGBA_BAD = [0.95, 0.15, 0.15, 0.45]


def robot_base_offset(cfg, robot_id):
    """Calibration (x, y, z mm) of the real robot base vs the table CAD: sim_config.json robot_base_offset_mm."""
    v = (cfg.get('robot_base_offset_mm') or {}).get(robot_id)
    if v is None:
        return (0.0, 0.0, float((cfg.get('robot_height_offset_mm') or {}).get(robot_id, 0.0)))   # older config key
    return tuple(float(x) for x in v)


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

    def __init__(self, model, home):
        d = mujoco.MjData(model)
        act = model.actuator('g_fingers_actuator').id
        jq = [model.jnt_qposadr[model.joint(n).id] for n in FINGER_JOINTS]
        gb = model.body('g_base').id
        pads = [model.geom(n).id for n in ('g_left_pad1', 'g_right_pad1')]
        half = model.geom_size[pads[0]][0]
        ctrl, gap, ang = [], [], []
        for c in np.linspace(0, 255, 11):
            mujoco.mj_resetData(model, d)
            d.qpos[:N_ARM] = d.ctrl[:N_ARM] = home
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



def finger_map(model, gripper, home):
    return FingerMap(model, home)


def collect_ghost_geoms(model, fingers, width_mm, home):
    """Gripper visual meshes + pads, as (kind, meshname|size, T_gbase_geom), fingers at width_mm."""
    data = mujoco.MjData(model)
    data.qpos[:N_ARM] = home
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

def tray_half_for(verts_w, com_w, cfg):
    """Inner half-size (x, y) of the target tray, in m, for a part with these (world-oriented) vertices.

    The part is placed with its centre of mass over place_xy, turned about the vertical by 0 or 90 deg first,
    so the tray is a square that holds the footprint's farthest X or Y extent from the COM, plus a margin.
    Parts that fit the original tray keep it (TRAY_HALF), so existing results do not change."""
    from world import TRAY_HALF
    reach = np.abs(verts_w[:, :2] - com_w[:2]).max()
    need = reach + float(cfg.get('tray_margin_m', 0.012))
    return (max(TRAY_HALF[0], need), max(TRAY_HALF[1], need)) if need > min(TRAY_HALF) else tuple(TRAY_HALF)

class CadPickPlace:
    def __init__(self, step_path, grasp, part_world_quat_wxyz, cfg, status, headless=False, snapshots=None,
                 robot=None):
        self.cfg, self.status, self.grasp = cfg, status, grasp
        try:
            self.robot = get_robot(robot)
        except ValueError as e:
            raise PlanError(str(e)) from None
        self.home = self.robot.home_qpos
        status.info['robot'] = self.robot.label
        self.headless, self.snap_dir = headless, snapshots
        self.viewer = self.renderer = None
        self.speed = float(cfg.get('speed', 1.0))

        status.update('converting', f'Converting {os.path.basename(step_path)} to meshes (cached after first run)...')
        # A mesh part (STL/OBJ, filled solid) has no known material: it gets its own density.
        density = (cfg.get('mesh_density_kg_m3', 2700) if is_mesh_file(step_path) else cfg.get('density_kg_m3', 7850))
        # Collision shape quality (sim_config collision_hulls): more, finer convex pieces keep cavities such as
        # a cup open, so a finger planned inside one is not reported as hitting the part.
        hulls = cfg.get('collision_hulls') or {}
        self.meta = convert(step_path, density=density, max_hulls=int(hulls.get('max_hulls', 24)),
                            threshold=float(hulls.get('threshold', 0.04)))
        status.info['part'] = {k: self.meta[k] for k in ('bbox_mm', 'mass_kg', 'volume_m3')}
        status.info['part']['hulls'] = len(self.meta['collision_hulls'])

        self.gripper = grasp.get('gripperId') or '2f85'
        if self.gripper not in GRIPPER_NAMES:
            raise PlanError(f"Unknown gripper '{self.gripper}' in the grasp (known: {', '.join(GRIPPER_NAMES)}).")
        status.info['gripper'] = GRIPPER_NAMES[self.gripper]
        self.tcp_offset_mm = float(grasp.get('tcpOffset', 145.0))
        self.pre_mm = float(grasp.get('preGraspDistance', 100.0))
        self.T_part_tcp = web_pose_to_T(grasp['tcpPose'])
        self.T_gb_tcp = T_gbase_tcp(self.tcp_offset_mm, self.gripper)

        # Place the part on the table in the same orientation it has in the web viewer.
        import trimesh
        verts = trimesh.load(os.path.join(self.meta['dir'], self.meta['visual_mesh']), force='mesh').vertices
        self.part_verts = np.asarray(verts)
        R = make_T([0, 0, 0], part_world_quat_wxyz)[:3, :3]
        rv = self.part_verts @ R.T
        c = (rv.min(0) + rv.max(0)) / 2
        # Pick station (sim_config pick_station): the part sits on its column, centre of mass over the axis.
        try:
            self.station = from_station_config(cfg)
        except (OSError, RuntimeError, ValueError) as e:
            raise PlanError(f'Pick station: {e}') from None
        self.support_z = 0.0
        if self.station[1] is not None:
            placement = self.station[1]
            cfg['pick_xy'] = placement['pick_xy']
            self.support_z = placement['support_z']
            c = R @ np.asarray(self.meta['com_m'])
            status.info['pick_station'] = {'pick_xy_mm': [round(v * 1000, 1) for v in placement['pick_xy']],
                                           'top_mm': round(self.support_z * 1000, 1),
                                           'hole_error_mm': placement['hole_error_mm']}
        px, py = cfg['pick_xy']
        part_pos = [px - c[0], py - c[1], self.support_z - rv.min(0)[2] + 0.002]
        # Fixture: the part made for it (pick_station.seated_part) goes on it as the two STEPs fit together, held
        # there by a weld until the gripper has closed. The orientation from the web viewer does not apply.
        self.seated = self.station[1] is not None and is_seated_part(self.station[1], step_path)
        if self.seated:
            # The part can only turn about the fixture axis (Z): that turn, from the web view, is kept.
            R_web = make_T([0, 0, 0], part_world_quat_wxyz)[:3, :3]
            tilt = float(np.degrees(np.arccos(np.clip(R_web[2, 2], -1.0, 1.0))))
            if tilt > 2.0:
                raise PlanError(f'The part is tilted {tilt:.0f} deg in the view, but on the fixture it can only turn '
                                'about the vertical (Z) axis. Reset its X/Y rotation in the Part Pose tab.')
            part_yaw = float(np.degrees(np.arctan2(R_web[1, 0], R_web[0, 0])))
            self.station = (self.station[0], dict(self.station[1], part_yaw_deg=part_yaw))
            part_pos, q_seat = seated_pose(self.station[1])
            part_world_quat_wxyz = np.asarray(q_seat)
            R = make_T([0, 0, 0], part_world_quat_wxyz)[:3, :3]
            rv = self.part_verts @ R.T
            self._check_seat_fit(part_pos, rv, self.station[1], status)
            # The grasp planner keeps the fingertips above this: here it is the real table top. The fixture is
            # in the collision check, and under an overhang of the part there is free space down to the table.
            self.support_z = 0.0
            status.info['pick_station']['seated_part'] = True
            status.info['pick_station']['part_yaw_deg'] = round(part_yaw, 1)
            print('Fixture: the part is seated on it (CAD fit), held until the gripper closes.', flush=True)


        self.tray_half = tray_half_for(rv, R @ np.asarray(self.meta['com_m']), cfg)
        status.info['tray_mm'] = [round(2000 * h) for h in self.tray_half]

        status.update('building', f'Building MuJoCo scene ({self.robot.label} + {GRIPPER_NAMES[self.gripper]} + part)...')
        build_kw = dict(cad_meta=self.meta, part_pos=part_pos, part_quat=part_world_quat_wxyz,
                        tray_xy=cfg['place_xy'], tray_half=self.tray_half,
                        tcp_offset_mm=self.tcp_offset_mm, gravcomp=True, fine_fingers=True,
                        robot=self.robot.id,
                        pick_station=(*self.station, self.seated) if self.station[1] is not None else None,
                        coupling_mm=float((cfg.get('gripper_coupling_mm') or {}).get(self.robot.id, {})
                                          .get(self.gripper, 0.0)),
                        base_offset_mm=robot_base_offset(cfg, self.robot.id))
        model0 = build(**build_kw)
        self.fingers = finger_map(model0, self.gripper, self.home)
        # The ghost shows the gripper as it grips: fingers at the contact width, not at the (wider) opening
        # used on the way in, so it looks like the real gripper once the fingers have closed on the part.
        ghost_width = grasp.get('measuredGraspWidth')
        if ghost_width is None or not np.isfinite(float(ghost_width)) or float(ghost_width) <= 0:
            ghost_width = GRIPPER_OPEN_MM[self.gripper]
        ghost_geoms = collect_ghost_geoms(model0, self.fingers, float(ghost_width), self.home)
        def extra(s):
            add_ghost(s, ghost_geoms, self.T_gb_tcp)

        self.model = build(**build_kw, extra=extra)
        self.data = mujoco.MjData(self.model)
        self.scratch = mujoco.MjData(self.model)

        m = self.model
        self.grip_act = m.actuator('g_fingers_actuator').id
        f = float(cfg.get('grip_force_limit', 5.0))
        m.actuator_forcerange[self.grip_act] = [-f, f]
        self.ghost_mocap = m.body_mocapid[m.body('ghost').id]
        self.ghost_geom_ids = [g for g in range(m.ngeom) if m.geom(g).name.startswith('ghost_')
                               and not m.geom(g).name.startswith('ghost_axis')]
        self.part_bid = m.body(PART_BODY).id
        self.robot_root = m.body(self.robot.base_bodies[0]).id
        self.ignore_env_bodies = {m.body(b).id for b in self.robot.base_bodies}
        self.pad_geoms = {g for g in range(m.ngeom) if m.geom(g).name.startswith(('g_left_pad', 'g_right_pad'))}

        self.pick = {**PICK_MOTION_DEFAULTS, **(cfg.get('pick_motion') or {})}
        self.home, self.T_home = self._central_home()

        self.data.qpos[:N_ARM] = self.home
        self.data.ctrl[:N_ARM] = self.home
        self.data.ctrl[self.grip_act] = GRIP_OPEN
        mujoco.mj_forward(m, self.data)
        self.path_points = []  # planned TCP path in world (drawn as dots)

    @staticmethod
    def _check_seat_fit(part_pos, rv, placement, status):
        """A part seated by its CAD frame must have been drawn in the fixture's frame. If its STEP has another
        origin it ends up floating over the fixture or sunk into the table: refuse with a clear message."""
        low = float(part_pos[2] + rv.min(0)[2])            # lowest point of the part, m above the table
        top = float(placement['support_z'])                # fixture top, m above the table
        status.info['pick_station']['part_low_mm'] = round(low * 1000, 1)
        if low < -0.002:
            raise PlanError(f'On the fixture this part would reach {-low * 1000:.0f} mm below the table top: its STEP '
                            'is not drawn in the fixture\'s coordinate frame. Export it from the assembly with the '
                            'fixture (same origin), or set pick_station.seat_all_parts to false.')
        if low > top + 0.003:
            raise PlanError(f'On the fixture this part would float {(low - top) * 1000:.0f} mm above the fixture top: its '
                            'STEP is not drawn in the fixture\'s coordinate frame. Export it from the assembly with the '
                            'fixture (same origin), or set pick_station.seat_all_parts to false.')

    def _central_home(self):
        """Home posture with the TCP at pick_motion.home_tcp_m, pointing straight down (the robot's own home
        posture seeds the IK, so the arm keeps its configuration). Falls back to that posture if unreachable."""
        m, s = self.model, self.scratch
        fixed = (self.pick.get('home_joints_deg') or {}).get(self.robot.id)
        if fixed:  # a home taught on the real robot (joint angles in degrees, as the pendant shows them)
            q = np.radians(np.asarray(fixed, dtype=float))
            s.qpos[:N_ARM] = q
            mujoco.mj_kinematics(m, s)
            return q, site_T(m, s, TCP_SITE)
        s.qpos[:N_ARM] = self.robot.home_qpos
        mujoco.mj_kinematics(m, s)
        T0 = site_T(m, s, TCP_SITE)
        z = np.array([0.0, 0.0, -1.0])
        x = T0[:3, 0] - (T0[:3, 0] @ z) * z
        x /= np.linalg.norm(x)
        T = np.eye(4)
        T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
        T[:3, 3] = self.pick['home_tcp_m']
        r = ik_solve(m, s, TCP_SITE, T, [self.robot.home_qpos])
        if not r.ok:
            print(f"Home TCP {self.pick['home_tcp_m']} is out of reach; using the robot's default home.", flush=True)
            return self.robot.home_qpos, T0
        return r.q, T

    def check_on_support(self, T_wp):
        """Raises PlanError when the settled part is no longer on the pick station (or table)."""
        if self.station[1] is None:
            if T_wp[2, 3] < -0.05:
                raise PlanError('The part rolled off the table while settling. Rotate it in the Part Pose tab '
                                'so it lies stably, then try again.')
            return
        com = T_wp[:3, :3] @ np.asarray(self.meta['com_m']) + T_wp[:3, 3]
        if T_wp[2, 3] > self.support_z - 0.05:   # still on it (a cup may rest with the column in its cavity)
            return
        info = self.station[0]
        column_d = 2 * info['column_radius_mm']
        if np.linalg.norm(com[:2] - np.asarray(self.cfg['pick_xy'])) < info['column_radius_mm'] / 1000.0:
            raise PlanError(f'The part slid down over the pick station column: its opening is wider than the '
                            f'column (Ø{column_d:.0f} mm). Turn it so it rests on the column top, or use a station '
                            f'with a wider top.')
        raise PlanError(f'The part fell off the pick station while settling: it cannot balance on the '
                        f'Ø{column_d:.0f} mm column top in this orientation. Rotate it in the Part Pose tab so a '
                        f'flat face rests on the column, then try again.')

    def pick_seconds(self, dist_m, speed_key, minimum=1.0):
        """Duration of a TCP move under cosine time scaling (peaks at pi/2 x the mean speed)."""
        return float(max(minimum, np.pi / 2 * dist_m / float(self.pick[speed_key])))

    # ---------------- stepping / display ----------------
    def open_viewer(self):
        if self.headless:
            if self.snap_dir:
                os.makedirs(self.snap_dir, exist_ok=True)
                self.renderer = mujoco.Renderer(self.model, 480, 640)
            return
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)
        self._aim(self.viewer.cam)

    def _draw_path(self, scn):
        """Planned TCP path as small dots (yellow = approach, green = grasp pose)."""
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

    def step(self, seconds):
        m, d = self.model, self.data
        n = max(1, int(seconds / m.opt.timestep))
        t0 = time.time()
        for i in range(n):
            mujoco.mj_step(m, d)
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

    def part_T(self, data=None):
        return body_T(self.model, data or self.data, PART_BODY)

    def part_min_z(self, T):
        return float((self.part_verts @ T[:3, :3].T + T[:3, 3])[:, 2].min())

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

    def idle_until_closed(self):
        if self.viewer is None:
            return
        try:
            while self.viewer.is_running():
                self.step(0.05)
        except KeyboardInterrupt:  # window closed by the user: normal end
            pass
