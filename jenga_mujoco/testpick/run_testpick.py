#!/usr/bin/env python3
"""
run_testpick.py - Scan-coverage prototype: the EC66 + 2F-85 picks the CAD part, holds it under a camera and
turns it view after view until the side surface has been scanned.

  ~/miniforge3/envs/mujoco_cad/bin/python jenga_mujoco/testpick/run_testpick.py
  ... --step "/path/with spaces/part (1).step"   other part
  ... --skip-pick                                start with the part already held at the scan point
  ... --no-attach                                friction-only grasp (no weld)
  ... --up z | --include-top-bottom              coverage options
  ... --headless                                 no window (prints the same report)

See README.md for every option.
"""
import argparse
import os
import sys
import time
import traceback

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from scene import Scene, load_configs, PART_BODY, PART_JOINT                   # noqa: E402  (adds jenga_mujoco/ to sys.path)
from coverage import Coverage, axis_vector                                     # noqa: E402
from viz import Viz                                                            # noqa: E402

from auto_pick_place import ARM_CLEARANCE, APPROACH_CLEARANCE                  # noqa: E402
from cad_pick_place import TCP_SITE, GRIP_CLOSED, GRIP_OPEN                   # noqa: E402
from collision_check import SceneCollision                                    # noqa: E402
from frames import make_T, inv, site_T, body_T, T_to_pos_quat                 # noqa: E402
from grasp_planner import GraspParams, generate_candidates                    # noqa: E402
from gripper_geometry import GripperGeometry                                  # noqa: E402
from ik import solve as ik_solve, N_ARM                                       # noqa: E402
from motion_planner import MotionPlanner, MotionError, joint_path_length      # noqa: E402

DEFAULT_CONFIG = os.path.join(HERE, 'config.json')


def rot(axis, deg):
    from scipy.spatial.transform import Rotation
    a = np.asarray(axis, dtype=float)
    return Rotation.from_rotvec(np.radians(deg) * a / np.linalg.norm(a)).as_matrix()


def align(a, b):
    """Smallest rotation taking unit vector a onto b."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        if c > 0:
            return np.eye(3)
        perp = np.cross(a, [1, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1, 0])
        return rot(perp, 180.0)
    return rot(v, np.degrees(np.arctan2(np.linalg.norm(v), c)))


class TestPick:
    def __init__(self, args):
        self.args = args
        self.cfg, sim_cfg = load_configs(args.config)
        step = args.step or self.cfg['default_step']
        if not os.path.isfile(step):
            raise SystemExit(f'Part file not found: {step!r}')
        self.sc = Scene(step, self.cfg, sim_cfg)
        self.m = self.sc.model
        self.d = mujoco.MjData(self.m)
        self.mo = self.cfg['motion']
        self.speed = float(args.speed)
        self.rng = np.random.default_rng(args.seed)
        self.viewer = self.viz = self.cov = None
        self._last_sync = self._last_draw = 0.0
        self._title = ('', '')
        self.views = []

    # ---------------- stepping / display ----------------
    def open_viewer(self):
        if self.args.headless:
            return
        import mujoco.viewer
        self.viewer = mujoco.viewer.launch_passive(self.m, self.d)
        cam = self.viewer.cam
        cam.lookat[:] = (self.sc.scan_point + np.array(self.sc.sim_cfg['pick_xy'] + [0.0])) / 2
        cam.distance, cam.azimuth, cam.elevation = 1.25, 215, -22

    def alive(self):
        return self.viewer is None or self.viewer.is_running()

    def sync(self, force=False):
        if self.viewer is None:
            return
        if not self.viewer.is_running():
            raise KeyboardInterrupt
        now = time.time()
        if self.viz is not None and (force or now - self._last_draw >= 1.0 / self.cfg['viz']['redraw_hz']):
            self.viz.draw(self.d)
            self._last_draw = now
        if force or now - self._last_sync >= 1 / 60:
            self.viewer.sync()
            self._last_sync = now

    def step(self, seconds):
        m, d = self.m, self.d
        n = max(1, int(round(seconds / m.opt.timestep)))
        t0 = time.time()
        for i in range(n):
            mujoco.mj_step(m, d)
            if self.viewer is not None and i % 8 == 0:
                self.sync()
                lag = (i + 1) * m.opt.timestep / self.speed - (time.time() - t0)
                if lag > 0:
                    time.sleep(lag)

    def move(self, q_waypoints, seconds, grip):
        """Arm through joint waypoints via the position actuators (d.ctrl), cosine time scaling (as
        CadPickPlace.move)."""
        d, m = self.d, self.m
        qs = [d.ctrl[:N_ARM].copy()] + [np.asarray(q, dtype=float) for q in q_waypoints]
        g0 = float(d.ctrl[self.sc.grip_act])
        n = max(2, int(seconds / (m.opt.timestep * 10)))
        for i in range(n):
            s = 0.5 * (1 - np.cos(np.pi * (i + 1) / n))
            u = s * (len(qs) - 1)
            k = min(int(u), len(qs) - 2)
            a = u - k
            d.ctrl[:N_ARM] = (1 - a) * qs[k] + a * qs[k + 1]
            d.ctrl[self.sc.grip_act] = g0 + (grip - g0) * s
            self.step(m.opt.timestep * 10)

    def joint_seconds(self, path, speed_key, minimum=1.0):
        q = [self.d.ctrl[:N_ARM].copy()] + list(path)
        return float(max(minimum, np.pi / 2 * joint_path_length(q) / float(self.mo[speed_key])))

    def say(self, title, detail=''):
        print(f'[{title}] {detail}', flush=True)
        if self.viz is not None:
            self.viz.text(title, detail)
        elif self.viewer is not None:
            self.viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                    title, detail)])

    # ---------------- poses ----------------
    def part_T(self, data=None):
        return body_T(self.m, data if data is not None else self.d, PART_BODY)

    def tcp_T(self, data=None):
        return site_T(self.m, data if data is not None else self.d, TCP_SITE)

    def set_part(self, data, T):
        p, q = T_to_pos_quat(T)
        a = self.sc.part_qadr
        data.qpos[a:a + 3] = p
        data.qpos[a + 3:a + 7] = q

    # ---------------- settle ----------------
    def settle(self):
        d = self.d
        d.qpos[:N_ARM] = d.ctrl[:N_ARM] = self.sc.home
        d.ctrl[self.sc.grip_act] = GRIP_OPEN
        mujoco.mj_forward(self.m, d)
        self.say('SETTLING', 'Letting the part settle on the table...')
        self.step(self.mo['settle_part_seconds'])
        for _ in range(18):
            v = d.qvel[self.sc.part_dadr:self.sc.part_dadr + 6]
            if np.linalg.norm(v) < 0.004:
                break
            self.step(0.25)
        T = self.part_T()
        if T[2, 3] < -0.05:
            raise SystemExit('The part fell off the table while settling.')
        return T

    def up_axis(self, T_wp):
        if self.args.up != 'auto':
            return axis_vector(self.args.up)
        return T_wp[:3, :3].T @ np.array([0.0, 0.0, 1.0])   # world Z in the part frame

    # ---------------- grasp (project planner) ----------------
    def choose_grasp(self, T_wp):
        """Best outside grasp of the project's grasp planner that the arm can reach, approach and lift
        collision-free (same checks as auto_pick_place.plan_candidate, simplified)."""
        import trimesh
        sc, m = self.sc, self.m
        self.say('PLANNING', 'Searching a grasp with the project grasp planner...')
        mesh = trimesh.load(os.path.join(sc.meta['dir'], sc.meta['visual_mesh']), force='mesh', process=True)
        geo = GripperGeometry(m, sc.fingers, sc.home)
        col = SceneCollision(m, sc.fingers, sc.robot, PART_BODY, PART_JOINT)
        planner = MotionPlanner(m, sc.scratch, TCP_SITE)
        self.col, self.planner = col, planner
        cands, rejects = generate_candidates(mesh.vertices, mesh.faces, None, None, T_wp, sc.meta['com_m'], geo,
                                             GraspParams(modes=('outside',)), table_z=sc.support_z)
        if not cands:
            raise SystemExit(f'No grasp found on this part: {rejects.most_common(3)}')
        pre = float(self.cfg['pre_grasp_mm']) / 1000.0
        lift_h = float(self.cfg['lift_height_m'])
        reasons = []
        for ci, cand in enumerate(cands[:40]):
            col.set_part(T_wp)
            T_grasp = T_wp @ cand.T_part_tcp
            T_pre = T_grasp @ make_T([0, 0, -pre], [1, 0, 0, 0])
            q_pre = planner.ik(T_pre, [sc.home])
            if q_pre is None:
                reasons.append('pre-grasp out of reach')
                continue
            approach = width_in = None
            for width in [geo.max_opening_mm] + [w for w in (cand.width_mm + 20, cand.width_mm + 8)
                                                 if w < geo.max_opening_mm]:
                free = lambda q, w=width: col.check(q, w, part_tol=-APPROACH_CLEARANCE) is None  # noqa: E731
                if not free(q_pre):
                    continue
                try:
                    approach = planner.line('approach', T_pre, T_grasp, q_pre, free)
                    width_in = width
                    break
                except MotionError:
                    continue
            if approach is None:
                reasons.append('approach blocked')
                continue
            q_grasp = approach[-1]
            if col.check(q_grasp, cand.width_mm + 2.0, ignore_pad_contact=True, part_tol=0.002) is not None:
                reasons.append('gripper hits the part at the grasp')
                continue
            carried = inv(cand.T_part_tcp)
            T_lift = T_grasp.copy()
            T_lift[2, 3] += lift_h
            try:
                lift = planner.line('lift', T_grasp, T_lift, q_grasp,
                                    lambda q: col.check(q, cand.width_mm, carried_T_tcp_part=carried) is None)
            except MotionError:
                reasons.append('cannot lift straight up')
                continue
            finally:
                col.set_part(T_wp)
            free_arm = lambda q, w=width_in: col.check(q, w, env_clearance=ARM_CLEARANCE) is None  # noqa: E731
            try:
                orient, travel = planner.orient_then_travel(sc.home, sc.T_home, q_pre, T_pre, free_arm)
                to_pre = list(orient) + list(travel)
            except MotionError:
                try:
                    to_pre = planner.connect('pre-grasp', sc.home, q_pre, free_arm)[1:]
                except MotionError:
                    reasons.append('no path to the pre-grasp')
                    continue
            tilt = np.degrees(np.arccos(np.clip(-T_grasp[2, 2], -1, 1)))
            print(f'[grasp] candidate {ci + 1}/{len(cands)}: {cand.width_mm:.1f} mm wide, approach {tilt:.0f} deg '
                  f'from vertical, fingers in at {width_in:.0f} mm', flush=True)
            return dict(cand=cand, to_pre=to_pre, approach=approach, lift=lift, width_in=width_in,
                        T_pre=T_pre, T_grasp=T_grasp)
        raise SystemExit('No grasp the arm can execute. Reasons: '
                         + ', '.join(f'{r} ({reasons.count(r)}x)' for r in sorted(set(reasons))))

    # ---------------- pick ----------------
    def hold(self):
        """After the fingers closed: weld (default) or friction only (+ noslip as auto_pick_place)."""
        if self.args.attach:
            self.sc.attach(self.d)
            print('[attach] part welded to g_base at its current pose (--no-attach for friction only)', flush=True)
        else:
            self.m.opt.noslip_iterations = int(self.mo['noslip_iterations'])

    def pick(self, g):
        self.say('PICK', 'Moving to the pre-grasp...')
        self.move(g['to_pre'], self.joint_seconds(g['to_pre'], 'free_speed_rad_s', 1.5),
                  self.sc.fingers.ctrl_for(g['width_in']))
        dist = float(np.linalg.norm(g['T_grasp'][:3, 3] - g['T_pre'][:3, 3]))
        self.say('PICK', 'Approaching...')
        self.move(g['approach'], max(2.0, np.pi / 2 * dist / self.mo['approach_speed_m_s']),
                  self.sc.fingers.ctrl_for(g['width_in']))
        self.step(0.3)
        self.say('PICK', 'Closing the gripper...')
        self.move([g['approach'][-1]], self.mo['grip_seconds'], GRIP_CLOSED)
        self.step(0.6)
        self.hold()
        z0 = self.part_T()[2, 3]
        self.say('PICK', 'Lifting...')
        self.move(g['lift'], max(2.0, np.pi / 2 * self.cfg['lift_height_m'] / 0.04), GRIP_CLOSED)
        self.step(0.3)
        rose = self.part_T()[2, 3] - z0
        if rose < 0.5 * self.cfg['lift_height_m']:
            raise SystemExit(f'The part was not lifted (rose {rose * 1000:.0f} mm). Try --attach.')

    def place_in_gripper_at_scan(self, g, R_base, c_body):
        """--skip-pick: arm, fingers and part set directly to 'held at the scan point', then welded."""
        sc, d = self.sc, self.d
        T_part = np.eye(4)
        T_part[:3, :3] = R_base
        T_part[:3, 3] = sc.scan_point - R_base @ c_body
        T_tcp = T_part @ g['cand'].T_part_tcp
        r = ik_solve(self.m, sc.scratch, TCP_SITE, T_tcp, [g['lift'][-1], sc.home])
        if not r.ok:
            raise SystemExit(f'--skip-pick: the scan point is out of reach with this grasp ({r}).')
        d.qpos[:N_ARM] = d.ctrl[:N_ARM] = r.q
        d.qvel[:] = 0
        sc.fingers.set_qpos(d.qpos, g['cand'].width_mm)
        d.ctrl[sc.grip_act] = GRIP_CLOSED
        self.set_part(d, T_part)
        mujoco.mj_forward(self.m, d)
        sc.attach(d)
        self.step(0.6)

    # ---------------- carrying the part ----------------
    def carry_path(self, T_part_from, T_part_to, T_part_tcp, q_from, width_mm):
        """Joint path that moves the part from one pose to another: its centre straight, its orientation
        slerped, small IK steps (like CadPickPlace.plan_inspection). Falls back to a collision-free joint path."""
        from scipy.spatial.transform import Rotation, Slerp
        mo = self.mo
        c = self.c_body
        ca = T_part_from[:3, :3] @ c + T_part_from[:3, 3]
        cb = T_part_to[:3, :3] @ c + T_part_to[:3, 3]
        Ra, Rb = T_part_from[:3, :3], T_part_to[:3, :3]
        ang = np.degrees(Rotation.from_matrix(Ra.T @ Rb).magnitude())
        n = max(1, int(np.ceil(max(np.linalg.norm(cb - ca) * 1000 / mo['step_mm'], ang / mo['step_deg']))))
        sl = Slerp([0, 1], Rotation.from_matrix([Ra, Rb]))
        carried = inv(T_part_tcp)
        free = lambda q: self.col.check(q, width_mm, carried_T_tcp_part=carried,  # noqa: E731
                                        carried_clearance=mo['carry_clearance_m']) is None
        qs, q_prev = [], np.asarray(q_from, dtype=float)
        try:
            for i in range(1, n + 1):
                T = np.eye(4)
                T[:3, :3] = sl(i / n).as_matrix()
                T[:3, 3] = ca + (cb - ca) * i / n - T[:3, :3] @ c
                r = ik_solve(self.m, self.sc.scratch, TCP_SITE, T @ T_part_tcp, [q_prev])
                if not r.ok or np.max(np.abs(r.q - q_prev)) > mo['max_joint_jump_rad'] or not free(r.q):
                    raise MotionError('step')
                qs.append(r.q)
                q_prev = r.q
            return qs
        except MotionError:
            r = ik_solve(self.m, self.sc.scratch, TCP_SITE, T_part_to @ T_part_tcp, [q_from])
            if not r.ok:
                return None
            try:
                return self.planner.connect('carry', q_from, r.q, free)[1:]
            except MotionError:
                return None

    # ---------------- views ----------------
    def candidates(self):
        nbv = self.cfg['nbv']
        yaws = np.arange(0.0, 360.0, float(nbv['yaw_step_deg']))
        return [(float(y), float(t)) for t in nbv['tilts_deg'] for y in yaws]

    def view_rotation(self, yaw, tilt):
        return rot(self.tilt_axis, tilt) @ rot([0, 0, 1], yaw) @ self.R_base

    def view_allowed(self, R):
        if self.args.include_top_bottom:
            return True
        lim = np.cos(np.radians(self.cfg['nbv']['min_top_angle_deg']))
        up_w = R @ self.cov.up
        return up_w @ self.to_cam < lim and -up_w[2] < lim

    def view_pose(self, yaw, tilt):
        R = self.view_rotation(yaw, tilt)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = self.sc.scan_point - R @ self.c_body
        return T

    def next_best_view(self, tried):
        """Scores every untried, allowed candidate by predicted new points (IK + mj_multiRay in a scratch
        MjData). Returns them best first: [(count, yaw, tilt, q_target)]."""
        m, s, d = self.m, self.sc.scratch, self.d
        T_part_tcp = inv(self.part_T()) @ self.tcp_T()        # CURRENT grasp transform
        q_now = d.qpos[:N_ARM].copy()
        scored = []
        for yaw, tilt in self.candidates():
            if (yaw, tilt) in tried:
                continue
            T_part = self.view_pose(yaw, tilt)
            if not self.view_allowed(T_part[:3, :3]):
                tried.add((yaw, tilt))
                continue
            r = ik_solve(m, s, TCP_SITE, T_part @ T_part_tcp, [q_now], n_random=2)
            if not r.ok:
                tried.add((yaw, tilt))
                continue
            s.qpos[:] = d.qpos
            s.qpos[:N_ARM] = r.q
            self.set_part(s, T_part)
            n = self.cov.predict(m, s)
            scored.append((n, -float(np.abs(r.q - q_now).sum()), yaw, tilt, r.q))
        scored.sort(key=lambda x: (x[0], x[1]), reverse=True)
        return [(n, yaw, tilt, q) for n, _, yaw, tilt, q in scored], T_part_tcp

    def do_scan(self, label):
        self.step(self.cfg['scan']['settle_seconds'])
        mujoco.mj_forward(self.m, self.d)
        res = self.cov.scan(self.m, self.d)
        if self.viz is not None:
            self.viz.show_beams(res['cam_pos'], res['hits_world'], self.cfg['scan']['beam_lines'], self.rng)
        detail = (f"{label}: {res['n_hits']} rays on the part, +{res['new']} new green points, "
                  f"coverage {res['coverage'] * 100:.1f}% ({self.cov.n_red()} red left)")
        self.say(f'SCAN {len(self.views) + 1}', detail)
        self.views.append(dict(label=label, new=res['new'], coverage=res['coverage']))
        self.step(self.cfg['scan']['show_seconds'])
        if self.viz is not None:
            self.viz.clear_beams()
        return res

    def scan_loop(self):
        nbv = self.cfg['nbv']
        target = float(nbv['target_coverage'])
        tried = set()
        while self.cov.coverage() < target and len(self.views) < int(nbv['max_views']):
            if not self.alive():
                raise KeyboardInterrupt
            self.say('NEXT VIEW', f'Predicting the next best view ({len(tried)} of {len(self.candidates())} '
                                  'candidates tried or ruled out)...')
            t0 = time.time()
            ranked, T_part_tcp = self.next_best_view(tried)
            moved = False
            for n, yaw, tilt, q in ranked:
                if n < int(nbv['min_new_points']):
                    break
                tried.add((yaw, tilt))
                path = self.carry_path(self.part_T(), self.view_pose(yaw, tilt), T_part_tcp,
                                       self.d.qpos[:N_ARM].copy(), self.hold_width)
                if path is None:
                    print(f'[nbv] yaw {yaw:.0f} tilt {tilt:+.0f}: no collision-free path, next candidate', flush=True)
                    continue
                print(f'[nbv] best: yaw {yaw:.0f} deg, tilt {tilt:+.0f} deg, predicts {n} new points '
                      f'({len(ranked)} reachable candidates scored in {time.time() - t0:.1f} s)', flush=True)
                self.say('MOVING', f'Turning the part to yaw {yaw:.0f} deg, tilt {tilt:+.0f} deg...')
                self.move(path, self.joint_seconds(path, 'carry_speed_rad_s', 1.5), GRIP_CLOSED)
                self.do_scan(f'yaw {yaw:.0f} tilt {tilt:+.0f}')
                moved = True
                break
            if not moved:
                print('[nbv] no reachable candidate view adds new points: stopping', flush=True)
                break

    # ---------------- run ----------------
    def run(self):
        args, sc = self.args, self.sc
        self.open_viewer()
        T_wp = self.settle()
        up = self.up_axis(T_wp)
        dist = float(np.linalg.norm(sc.cam_eye - sc.scan_point))
        self.cov = Coverage(self.m, sc.part_bid, sc.part_geom, sc.cam_id, sc.cam_body, self.cfg['scan'], up,
                            args.include_top_bottom, dist, seed=args.seed)
        mesh = self.cov.mesh
        self.c_body = (mesh.vertices.min(0) + mesh.vertices.max(0)) / 2      # part centre (bbox), body frame
        if self.viewer is not None:
            self.viz = Viz(self.viewer, self.cov, self.cfg['viz'], cone_length=dist * 1.25)
            self.sync(force=True)

        # Reference orientation: the settled pose, turned so UP points straight up (identity for --up auto).
        R0 = T_wp[:3, :3]
        self.R_base = align(R0 @ up, np.array([0.0, 0.0, 1.0])) @ R0
        view = sc.scan_point - sc.cam_eye
        vh = np.array([view[0], view[1], 0.0])
        vh /= np.linalg.norm(vh)
        self.tilt_axis = np.cross(vh, [0.0, 0.0, 1.0])     # tilt > 0 turns the top towards the camera
        self.to_cam = -view / np.linalg.norm(view)

        g = self.choose_grasp(T_wp)
        self.hold_width = g['cand'].width_mm
        if args.skip_pick:
            self.say('SKIP PICK', 'Part placed in the gripper at the scan point (welded)')
            self.place_in_gripper_at_scan(g, self.R_base, self.c_body)
        else:
            self.pick(g)
        self.say('SCANNING', f'Target {self.cfg["nbv"]["target_coverage"] * 100:.0f}% of '
                             f'{int(self.cov.active.sum())} target points')
        self.scan_loop()
        return self.final_report()

    def final_report(self):
        gripper_bodies = {b for b in range(self.m.nbody) if self.m.body(b).name.startswith('g_')}
        rep = self.cov.report(self.m, self.d, gripper_bodies)
        lines = [f"coverage {rep['coverage'] * 100:.1f}% of {rep['active']} target points after "
                 f"{len(self.views)} views",
                 f"{rep['red']} red points remain, {rep['red_under_gripper']} of them covered by the gripper fingers",
                 f"{rep['excluded']} top/bottom points ignored"]
        print('\n========== FINAL COVERAGE REPORT ==========', flush=True)
        for i, v in enumerate(self.views, 1):
            print(f"  view {i:2d}: {v['label']:<20s} +{v['new']:4d} points -> {v['coverage'] * 100:5.1f}%")
        for line in lines:
            print('  ' + line)
        print('===========================================', flush=True)
        self.say('DONE', '  |  '.join(lines) + '  |  close the window to exit')
        return rep

    def idle(self):
        if self.viewer is None:
            return
        try:
            while self.viewer.is_running():
                self.step(0.05)
        except KeyboardInterrupt:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--step', help='STEP/STL part file (default: config.json default_step). Quote paths with spaces.')
    ap.add_argument('--config', default=DEFAULT_CONFIG)
    ap.add_argument('--attach', dest='attach', action='store_true', default=True,
                    help='weld the part to the gripper once it has closed (default)')
    ap.add_argument('--no-attach', dest='attach', action='store_false', help='friction-only grasp')
    ap.add_argument('--skip-pick', action='store_true', help='start with the part held (welded) at the scan point')
    ap.add_argument('--up', choices=['auto', 'x', 'y', 'z'], default='auto',
                    help="part's up axis in its own frame (auto = world Z of the settled pose)")
    ap.add_argument('--include-top-bottom', action='store_true', help='scan the top and bottom faces too')
    ap.add_argument('--headless', action='store_true', help='no viewer window')
    ap.add_argument('--speed', type=float, default=1.0, help='simulation speed vs real time (viewer only)')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    demo = None
    try:
        demo = TestPick(args)
        demo.run()
        demo.idle()
        return 0
    except KeyboardInterrupt:
        print('Viewer closed.', flush=True)
        return 0
    except SystemExit as e:
        if e.code not in (None, 0) and not isinstance(e.code, int):
            print(f'ERROR: {e.code}', flush=True)
            if demo is not None:
                demo.say('STOPPED', str(e.code))
                demo.idle()
            return 1
        raise
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        if demo is not None and demo.viewer is not None:
            demo.idle()
        return 1


if __name__ == '__main__':
    sys.exit(main())
