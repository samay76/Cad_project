#!/usr/bin/env python3
"""
auto_pick_place.py - Geometry-aware pick & place: click a point on a STEP part, the robot does the rest.

Launched by the web app's Auto Pick & Place panel (vite-plugins/mujocoBridge.ts), or by hand:

  # one finger on each marked point: x,y,z,nx,ny,nz in the part frame (mm, surface normal)
  python auto_pick_place.py --step part.step --point1=0,0,10,0,-1,0 --point2=0,30,10,0,1,0 --mode plan --plan plan.json
  # or a single point: the grasp is searched near it
  python auto_pick_place.py --step part.step --click 0,20,0 --normal 0,1,0 --mode plan --plan plan.json
  python auto_pick_place.py --step part.step --plan plan.json --mode preview     # watch it, no physics
  python auto_pick_place.py --step part.step --plan plan.json --mode execute     # run it with physics
  add --robot ec66 / --gripper hande, and --headless --snapshots out/ to run without a window

Modes:
  plan     settle the part, find grasps, plan the motions, write plan.json. No window.
  preview  open MuJoCo and play the planned motion kinematically (arm, gripper, part; no physics).
  execute  run the planned motion with full physics and report what actually happened.

Each module is responsible for one thing:
  grasp_planner.py     candidate grasps from the part geometry
  gripper_geometry.py  where the gripping faces are, measured from the gripper model
  collision_check.py   is this arm posture collision-free (part on the table, or carried)
  motion_planner.py    straight TCP lines and RRT-Connect through free space
  ik.py                inverse kinematics
  cad_pick_place.py    the MuJoCo scene, stepping, viewer and ghost gripper (CadPickPlace, reused here)

Sequence: home -> pre-grasp -> grasp -> close -> lift -> inspection (camera on the table, set up in
sim_config.json) -> transfer -> place -> open -> retreat -> home.
"""
import argparse
import dataclasses
import json
import os
import sys
import time
import traceback
from collections import Counter

import numpy as np
import mujoco

from cad_part import PART_JOINT
from cad_pick_place import CadPickPlace, PlanError, Status, GRIP_CLOSED, GRIP_OPEN, GRIPPER_NAMES, TCP_SITE
from collision_check import SceneCollision
from frames import T_to_pos_quat, inv, make_T, pose_error, quat_xyzw_to_wxyz, site_T
from grasp_planner import GraspParams, PointsError, gap_at_click, generate_candidates, grasps_from_points
from gripper_geometry import GripperGeometry
from ik import N_ARM
from motion_planner import MotionError, MotionPlanner, joint_path_length, resample
from robots import ROBOTS

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TCP_MM = {'2f85': 145.0, 'hande': 146.5}
PLAN_VERSION = 1
SETTLE_MAX_S = 6.0
SETTLE_SPEED = 0.002             # m/s and rad/s: the part counts as at rest below this
FREE_SPEED, CARRY_SPEED = 0.8, 0.5   # rad/s, peak joint speed in free space and carrying the part
ARM_CLEARANCE = 0.008            # m from the environment, in free space
CARRY_CLEARANCE = 0.006          # m of carried part from the environment, in transfer
APPROACH_CLEARANCE = 0.001       # m of fingers from the part on the way to the grasp, when it can be had
NOSLIP_ITERATIONS = 5
DURATION_PENALTY = 0.05          # grasp score points per second of motion
PLACE_YAWS_DEG = (0, 90, -90, 180, 45, -45, 135, -135)

PHASE_LABELS = {
    'pre-grasp': 'Moving to the pre-grasp position',
    'approach': 'Approaching the grasp',
    'grip': 'Closing the gripper',
    'lift': 'Lifting the part',
    'inspect': 'Showing the part to the camera',
    'transfer': 'Carrying the part to the target',
    'place': 'Lowering the part',
    'release': 'Opening the gripper',
    'retreat': 'Retreating',
    'home': 'Returning home',
}


def pose_json(T):
    """T (m) -> web Pose (mm, quaternion xyzw)."""
    p, q = T_to_pos_quat(T)
    p = p * 1000.0
    return {'position': {'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2])},
            'orientation': {'x': float(q[1]), 'y': float(q[2]), 'z': float(q[3]), 'w': float(q[0])}}


def rpy_json(T):
    """XYZ Euler angles in degrees, the convention of the web tool's PoseMath."""
    from scipy.spatial.transform import Rotation
    r, p, y = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    return {'roll': float(r), 'pitch': float(p), 'yaw': float(y)}


def part_reason(hit):
    if hit.kind == 'environment':
        return f"gripper or arm would hit '{hit.b}'"
    if hit.kind == 'part':
        return 'finger would hit the part' if hit.pad else 'gripper body would hit the part'
    if hit.kind == 'self':
        return 'arm would hit itself'
    return f"carried part would hit '{hit.b}'"


class CandidateRejected(Exception):
    """This grasp does not work; the message is the tally key shown to the user."""


class AutoPickPlace(CadPickPlace):
    def __init__(self, step_path, req, q_part, cfg, status, headless=False, snapshots=None, feed_dir=None):
        gripper = req.get('gripperId') or '2f85'
        if gripper not in GRIPPER_NAMES:
            raise PlanError(f"Unknown gripper '{gripper}' (known: {', '.join(GRIPPER_NAMES)}).")
        # The scene is built around a grasp; before planning there is none yet, so a neutral one stands in.
        grasp = req.get('grasp') or {'tcpPose': {'position': {'x': 0, 'y': 0, 'z': 0},
                                                 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}}
        grasp = {'gripperId': gripper,
                 'tcpOffset': float(req.get('tcpOffset') or DEFAULT_TCP_MM[gripper]),
                 'preGraspDistance': float(req.get('preGraspDistance') or 100.0),
                 **grasp}
        super().__init__(step_path, grasp, q_part, cfg, status, headless=headless, snapshots=snapshots,
                         feed_dir=feed_dir, robot=req.get('robotId'))
        # MuJoCo's soft contacts let a firmly held part creep along the fingers (~0.1 mm/s): over a long camera
        # sequence it walks off the pad and falls. This removes that drift only; a weak grip still slips.
        self.model.opt.noslip_iterations = NOSLIP_ITERATIONS
        self.part_quat = np.asarray(q_part, dtype=float)
        self.free_joint_dof = int(self.model.jnt_dofadr[self.model.joint(PART_JOINT).id])
        self.free_joint_adr = int(self.model.jnt_qposadr[self.model.joint(PART_JOINT).id])

    # ---------------- settling ----------------
    def settle(self):
        """Lets the part come to rest on the table. Everything is planned from this pose."""
        self.step(self.cfg['settle_seconds'])
        waited = self.cfg['settle_seconds']
        while waited < SETTLE_MAX_S:
            v = self.data.qvel[self.free_joint_dof:self.free_joint_dof + 6]
            if np.linalg.norm(v[:3]) < SETTLE_SPEED and np.linalg.norm(v[3:]) < SETTLE_SPEED:
                break
            self.step(0.25)
            waited += 0.25
        T = self.part_T()
        if T[2, 3] < -0.05:
            raise PlanError('The part rolled off the table while settling. Rotate it in the Part Pose tab '
                            'so it lies stably, then try again.')
        return T

    # ---------------- planning ----------------
    def compute_plan(self, click_point, click_normal, n_plans=3, points=None):
        started = time.time()
        st = self.status
        st.update('settling', 'Letting the part settle on the table...')
        T_start = self.part_T()
        T_wp = self.settle()
        drift = np.linalg.norm(T_wp[:2, 3] - T_start[:2, 3]) * 1000.0
        tipped = pose_error(T_start, T_wp)[1]

        st.update('grasp', 'Analysing the part geometry for grasps...')
        import trimesh
        mesh = trimesh.load(os.path.join(self.meta['dir'], self.meta['visual_mesh']), force='mesh', process=True)
        self.geo = GripperGeometry(self.model, self.fingers, self.home)
        self.collision = SceneCollision(self.model, self.fingers, self.robot, 'cad_part', PART_JOINT)
        self.planner = MotionPlanner(self.model, self.scratch, TCP_SITE)
        params = self.params = GraspParams(**(self.cfg.get('auto_grasp') or {}))
        if points is not None:
            # The user chose where the fingers go: plan for exactly that, or say why not.
            try:
                candidates, rejects, note = grasps_from_points(mesh.vertices, mesh.faces, points, T_wp, self.geo, params)
            except PointsError as e:
                raise PlanError(str(e)) from None
            st.update('grasp', f'{len(candidates)} ways to reach both points; checking them on the robot...')
            plans, rejected = self._plans_from(candidates, T_wp, n_plans)
            if not plans:
                why = '; '.join(f'{k} ({v}x)' for k, v in (rejected or rejects).most_common(3))
                raise PlanError(f'The gripper cannot reach both marked points with the {GRIPPER_NAMES[self.gripper]}. '
                                f'Main reasons: {why}. Try other points or the other gripper.')
        else:
            # Clicking the wall of a bore means: enter it with closed fingers and open them against the wall.
            wants_inside = 'inside' in params.modes and gap_at_click(mesh.vertices, mesh.faces, click_point, click_normal, params)
            first = dataclasses.replace(params, modes=('inside',)) if wants_inside else params
            candidates, rejects = generate_candidates(
                mesh.vertices, mesh.faces, click_point, click_normal, T_wp, self.meta['com_m'], self.geo, first)
            st.update('grasp', f"{len(candidates)} {'inside-grip ' if wants_inside else ''}grasp candidates from the "
                               'geometry; checking them on the robot...')
            plans, rejected = self._plans_from(candidates, T_wp, n_plans)

            note = ''
            if not plans and click_point is not None:
                why = (rejected or rejects).most_common(1)
                reason = f' ({why[0][0]})' if why else ''
                note = (f'No inside grip works in this bore{reason}, so this grasp is elsewhere on the part. ' if wants_inside
                        else f'No grasp works at the selected point{reason}, so this one is elsewhere on the part. ')
                st.update('grasp', 'Nothing works at the selected point; searching the whole part...')
                candidates, more = generate_candidates(mesh.vertices, mesh.faces, None, None, T_wp, self.meta['com_m'],
                                                       self.geo, params)
                rejects.update(more)
                plans, more_rejected = self._plans_from(candidates, T_wp, n_plans)
                rejected.update(more_rejected)

        if not plans:
            raise PlanError(self._failure_message(rejected, rejects, candidates))
        # Between grasps of similar quality, prefer the one the arm reaches without a detour.
        plans.sort(key=lambda p: -(p['score'] - DURATION_PENALTY * p['durationS']))

        return {
            'version': PLAN_VERSION,
            'robot': self.robot.id, 'robotLabel': self.robot.label,
            'gripper': self.gripper, 'gripperLabel': GRIPPER_NAMES[self.gripper],
            'tcpOffset': self.tcp_offset_mm,
            'click': None if click_point is None else (np.asarray(click_point) * 1000).tolist(),
            'points': None if points is None else [(p * 1000).tolist() for p, _ in points],
            'awayFromClick': bool(note),
            'note': note,
            'part': {'T_world_part': T_wp.tolist(), 'tippedDeg': round(tipped, 1), 'driftMm': round(float(drift), 1),
                     'massKg': self.meta['mass_kg'], 'sizeMm': self.meta['bbox_mm']['size']},
            'scene': self._scene_json(T_wp),
            'world_in_part': pose_json(inv(T_wp)),
            'plans': plans,
            'rejected': dict(rejected.most_common(8)),
            'seconds': round(time.time() - started, 1),
        }

    def _plans_from(self, candidates, T_wp, n_plans):
        """Complete motions for the best candidates that work, and a tally of why the others did not."""
        plans, rejected = [], Counter()
        for i, cand in enumerate(candidates):
            try:
                plans.append(self.plan_candidate(cand, T_wp))
                self.status.update('motion', f'Found {len(plans)}/{n_plans} complete motions '
                                             f'(checked {i + 1}/{len(candidates)} grasps)...')
            except CandidateRejected as e:
                rejected[str(e)] += 1
            if len(plans) == n_plans:
                break
        return plans, rejected

    def _failure_message(self, on_robot, geometric, candidates):
        """Grasps that failed on the robot explain more than samples that failed on geometry."""
        rejects = on_robot or geometric
        size = self.meta['bbox_mm']['size']
        opening = self.geo.max_opening_mm
        head = (f'No workable grasp on this part ({size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm) with the '
                f'{GRIPPER_NAMES[self.gripper]} (opens {opening:.0f} mm).')
        why = '; '.join(f'{k} ({v}x)' for k, v in rejects.most_common(3))
        if not candidates and min(size) > opening:
            return f'{head} Every dimension is wider than the gripper opens.'
        return f'{head} Main reasons: {why}. Try another point on the part or the other gripper.'

    def _scene_json(self, T_wp):
        """Static boxes of the scene (world frame, mm) so the web preview can draw the workspace."""
        m = self.model
        boxes = []
        for g in range(m.ngeom):
            name = m.geom(g).name
            if m.geom_bodyid[g] == 0 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and \
                    (name == 'table_top' or name.startswith('box_')):
                boxes.append({'name': name, 'pos': (m.geom_pos[g] * 1000).tolist(),
                              'halfSize': (m.geom_size[g] * 1000).tolist()})
        return {'boxes': boxes, 'placeXY': [float(v) * 1000 for v in self.cfg['place_xy']]}

    def plan_candidate(self, cand, T_wp):
        """Complete motion for one grasp, or CandidateRejected."""
        col, planner = self.collision, self.planner
        col.set_part(T_wp)
        T_part_tcp = cand.T_part_tcp
        T_grasp = T_wp @ T_part_tcp
        inside = cand.mode == 'inside'
        pre_mm = float(self.pre_mm)
        T_pre = T_grasp @ make_T([0, 0, -pre_mm / 1000.0], [1, 0, 0, 0])
        q_grasp_seed = planner.ik(T_pre, [self.home])
        if q_grasp_seed is None:
            raise CandidateRejected('pre-grasp position out of reach')

        # Fingers on the way in. Wide open is safest; narrower when a neighbouring feature blocks it.
        if inside:
            openings = [0.0]
        else:
            opening_max = self.geo.max_opening_mm
            openings = [opening_max] + [w for w in (cand.width_mm + 20.0, cand.width_mm + 8.0) if w < opening_max]

        # First look for an opening that clears the part, so the fingers don't nudge it on the way down.
        # Only if none does, accept one that brushes it within the collision tolerance.
        approach, q_pre, width_in, last = None, None, None, None
        for part_tol in (-APPROACH_CLEARANCE, 0.001):
            for width in openings:
                free = lambda q, w=width, tol=part_tol: col.check(q, w, part_tol=tol) is None  # noqa: E731
                hit = col.check(q_grasp_seed, width, part_tol=min(part_tol, 0.0))
                if hit is not None:
                    last = part_reason(hit)
                    continue
                try:
                    approach = planner.line('approach', T_pre, T_grasp, q_grasp_seed, free)
                    q_pre, width_in = q_grasp_seed, width
                    break
                except MotionError as e:
                    last = 'approach path blocked' if 'collides' in str(e) else 'grasp position out of reach'
            if approach is not None:
                break
        if approach is None:
            raise CandidateRejected(last or 'approach path blocked')
        q_grasp = approach[-1]
        # A bore is a tight fit: the closed fingers need real clearance at this depth, not just no overlap.
        if inside and col.check(q_grasp, 0.0, part_tol=-self.params.finger_fit_m) is not None:
            raise CandidateRejected('closed fingers are too tight in the opening at this depth')

        # Fingers at the grip: the pads touch the part by design, nothing else may.
        grip_width = width_in if inside else cand.width_mm + 2.0
        hit = col.check(q_grasp, grip_width, ignore_pad_contact=True, part_tol=0.002)
        if hit is not None and not inside:
            raise CandidateRejected(part_reason(hit))

        T_tcp_part = inv(T_part_tcp)
        hold = self._inside_hold_width(q_grasp) if inside else cand.width_mm
        def free_carry(clearance):
            return lambda q: col.check(q, hold, carried_T_tcp_part=T_tcp_part, carried_clearance=clearance) is None

        # Lift straight up. The part starts on the table, so it only has to stop penetrating.
        T_lift = T_grasp.copy()
        T_lift[2, 3] += self.cfg['lift_height_m']
        try:
            lift = planner.line('lift', T_grasp, T_lift, q_grasp, free_carry(0.0))
        except MotionError:
            raise CandidateRejected('cannot lift straight up') from None
        finally:
            col.set_part(T_wp)

        placed = self._plan_place(T_wp, T_part_tcp, lift[-1], free_carry, hold, width_in, inside)
        col.set_part(T_wp)

        # Free-space move to the pre-grasp, with the part still on the table.
        def free_arm(q):
            return col.check(q, width_in, env_clearance=ARM_CLEARANCE) is None
        try:
            pre_path = planner.connect('pre-grasp', self.home, q_pre, free_arm)
        except MotionError:
            raise CandidateRejected('no collision-free path to the pre-grasp') from None

        # Last, so this time is only spent on grasps that otherwise work. It never rules a grasp out.
        self.T_part_tcp = T_part_tcp
        inspect, skipped, transfer = self._plan_inspection_stage(T_wp, lift[-1], placed, hold, free_carry)
        col.set_part(T_wp)
        # Every waypoint gets the same time, so they must be evenly spaced or the big steps become speed spikes
        # that throw the part out of the fingers. move_seconds is a minimum; long turns take longer.
        move_s = self.insp.get('move_seconds', 2.0)
        inspect_phases, q_prev = [], lift[-1]
        for label, qs in inspect:
            path = resample([q_prev, *qs])
            inspect_phases.append(self._phase('inspect', path, hold, True, CARRY_SPEED, minimum=move_s,
                                              label=f'Showing the part to the camera: {label}', view=label))
            q_prev = path[-1]

        phases = [
            self._phase('pre-grasp', pre_path, width_in, False, FREE_SPEED),
            self._phase('approach', approach, width_in, False, CARRY_SPEED, minimum=1.5),
            self._phase('grip', [q_grasp], width_in, False, seconds=1.0),
            self._phase('lift', lift, hold, True, CARRY_SPEED, minimum=1.5),
            *inspect_phases,
            self._phase('transfer', transfer, hold, True, CARRY_SPEED, minimum=2.5),
            self._phase('place', placed['down'], hold, True, CARRY_SPEED, minimum=1.5),
            self._phase('release', [placed['down'][-1]], placed['release_width'], False, seconds=1.0),
            self._phase('retreat', placed['retreat'], placed['release_width'], False, CARRY_SPEED, minimum=1.5),
            self._phase('home', placed['home'], self.geo.max_opening_mm, False, FREE_SPEED, minimum=1.5),
        ]
        duration = sum(p['seconds'] for p in phases)
        for p in phases:
            self._add_tcp_samples(p, T_wp)
        tcp = pose_json(T_part_tcp)
        approach_world = T_grasp[:3, 2]
        tilt = float(np.degrees(np.arccos(np.clip(-approach_world[2], -1, 1))))
        return {
            'grasp': {
                'id': f'auto_{int(time.time() * 1000)}_{len(phases)}', 'name': 'Auto Grasp',
                'tcpPose': tcp, 'rpy': rpy_json(T_part_tcp), 'openingWidth': width_in,
                'measuredGraspWidth': cand.width_mm if not inside else width_in,
                'contactPoints': {'left': self._mm_vec(cand.contacts[0]), 'right': self._mm_vec(cand.contacts[1])},
                'preGraspDistance': pre_mm, 'tcpOffset': self.tcp_offset_mm, 'gripMode': cand.mode,
                'approachAxis': {'x': 0, 'y': 0, 'z': 1}, 'closingAxis': {'x': 1, 'y': 0, 'z': 0},
                'gripperId': self.gripper, 'createdAt': int(time.time() * 1000),
            },
            'mode': cand.mode, 'widthMm': round(cand.width_mm, 1), 'approachTiltDeg': round(tilt, 1),
            'score': round(cand.score, 2), 'terms': {k: round(v, 2) for k, v in cand.terms.items()},
            'placeYawDeg': placed['yaw'], 'T_tcp_part': T_tcp_part.tolist(), 'T_wp_placed': placed['T_wp'].tolist(),
            'durationS': round(duration, 1), 'phases': phases, 'inspectionViews': len(inspect),
            'inspectionSkipped': skipped,
        }

    @staticmethod
    def _mm_vec(p):
        return {'x': float(p[0]) * 1000, 'y': float(p[1]) * 1000, 'z': float(p[2]) * 1000}

    def _plan_inspection_stage(self, T_wp, q_lifted, placed, hold, free_carry):
        """Camera views after the lift, and the transfer from the last view to above the target.
        Returns (views [(label, joint path)], skipped reasons, transfer path). Without usable views the transfer
        planned from the lift stands. Views are consecutive moves, so a view that fails ends the sequence."""
        planned, skipped = self.plan_inspection(T_wp, q_lifted, hold)
        check = free_carry(0.0)
        views = []
        for i, (label, qs) in enumerate(planned):
            if not all(check(q) for q in qs):
                skipped += [f'{name} (the part would hit something)' for name, _ in planned[i:]]
                break
            views.append((label, qs))
        while views:
            q_from = views[-1][1][-1]
            try:
                return views, skipped, self._transfer_after_inspection(q_from, q_lifted, placed, hold, free_carry)
            except MotionError:
                skipped.append(f'{views.pop()[0]} (no path to the target from this view)')
        return views, skipped, placed['transfer']

    def _transfer_after_inspection(self, q_from, q_lifted, placed, hold, free_carry):
        """Turns the part back in place first (plan_transfer), so a tilted view does not swing it around on the way."""
        free = free_carry(CARRY_CLEARANCE)
        route = self.plan_transfer(q_from, placed['T_above'], q_lifted, hold)
        if len(route) > 1 and all(free(q) for q in route):
            try:
                ending = self.planner.connect('transfer', route[-1], placed['q_above'], free)
                return resample([q_from, *route, *ending])
            except MotionError:
                pass  # try the free-space move below
        return self.planner.connect('transfer', q_from, placed['q_above'], free)

    def _inside_hold_width(self, q_grasp, step_mm=1.0):
        """Opening at which fingers inside a bore touch its walls, found on the real shapes at this depth."""
        width = 0.0
        while width < self.geo.max_opening_mm:
            hit = self.collision.check(q_grasp, width + step_mm, part_tol=0.0)
            if hit is not None and hit.kind == 'part':
                break
            width += step_mm
        return width

    def _plan_place(self, T_wp, T_part_tcp, q_lifted, free_carry, hold, width_in, inside):
        """Target pose, transfer, lowering, release and retreat. Tries other yaws if a pose does not work."""
        cfg, col, planner = self.cfg, self.collision, self.planner
        com = T_wp[:3, :3] @ np.asarray(self.meta['com_m']) + T_wp[:3, 3]
        place_xy = np.asarray(cfg['place_xy'], dtype=float)
        floor = cfg['place_floor_z'] + cfg['place_clearance_m']
        lift = cfg['lift_height_m']
        last = 'target position out of reach'

        for yaw in PLACE_YAWS_DEG:
            c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            T_new = np.eye(4)
            T_new[:3, :3] = Rz @ T_wp[:3, :3]
            T_new[:3, 3] = Rz @ (T_wp[:3, 3] - com) + np.r_[place_xy, com[2]]
            T_new[2, 3] += floor - self.part_min_z(T_new)
            T_place = T_new @ T_part_tcp
            T_above = T_place.copy()
            T_above[2, 3] += lift
            try:
                q_above = planner.ik(T_above, [q_lifted])
                if q_above is None:
                    raise CandidateRejected('target position out of reach')
                down = planner.line('place', T_above, T_place, q_above, free_carry(0.0))
                release_width = self._release_width(down[-1], T_new, hold, width_in, inside)
                back = T_place @ make_T([0, 0, -self.pre_mm / 1000.0], [1, 0, 0, 0])
                col.set_part(T_new)
                free_open = lambda q: col.check(q, release_width) is None  # noqa: E731
                retreat = planner.line('retreat', T_place, back, down[-1], free_open)
                transfer = planner.connect('transfer', q_lifted, q_above, free_carry(CARRY_CLEARANCE))
                col.set_part(T_new)
                free_home = lambda q: col.check(q, self.geo.max_opening_mm, env_clearance=ARM_CLEARANCE) is None  # noqa: E731
                home = planner.connect('home', retreat[-1], self.home, free_home)
            except CandidateRejected as e:
                last = str(e)
                continue
            except MotionError as e:
                last = ('part does not fit at the target' if "'place'" in str(e) else
                        f'no path for {str(e).split(chr(39))[1]}')
                continue
            return dict(transfer=transfer, down=down, retreat=retreat, home=home, release_width=release_width,
                        T_above=T_above, q_above=q_above,
                        T_wp=T_new, yaw=yaw)
        raise CandidateRejected(last)

    def _release_width(self, q_placed, T_new, hold, width_in, inside):
        """Widest useful opening at the target that hits neither the placed part nor the tray."""
        col = self.collision
        col.set_part(T_new)
        if inside:
            return width_in
        for width in (min(width_in, hold + 20.0), hold + 10.0, hold + 5.0):
            if width <= self.geo.max_opening_mm and col.check(q_placed, width) is None:
                return width
        raise CandidateRejected('fingers cannot open at the target')

    def _phase(self, name, qs, width, attached, speed=None, seconds=None, minimum=0.5, label=None, view=None):
        qs = [np.asarray(q, dtype=float) for q in qs]
        if seconds is None:
            # Cosine time scaling peaks at pi/2 x the mean speed.
            seconds = max(minimum, np.pi / 2 * joint_path_length(qs) / speed)
        return {'name': name, 'label': label or PHASE_LABELS[name], 'q': [q.round(5).tolist() for q in qs],
                'width': float(width), 'attached': bool(attached), 'seconds': round(float(seconds), 2), 'view': view}

    def _add_tcp_samples(self, phase, T_wp, max_samples=40):
        """TCP poses in the part's starting frame (mm, xyzw), which is what the web viewer can draw."""
        qs = phase['q']
        step = max(1, len(qs) // max_samples)
        idx = list(range(0, len(qs), step))
        if idx[-1] != len(qs) - 1:
            idx.append(len(qs) - 1)
        T_pw = inv(T_wp)
        samples = []
        for i in idx:
            T = site_T_of(self.model, self.scratch, qs[i])
            samples.append(pose_json(T_pw @ T))
        phase['tcp'] = samples

    # ---------------- preview (kinematic) ----------------
    def _set_part_qpos(self, T):
        p, q = T_to_pos_quat(T)
        self.data.qpos[self.free_joint_adr:self.free_joint_adr + 3] = p
        self.data.qpos[self.free_joint_adr + 3:self.free_joint_adr + 7] = q

    def _world_path(self, plan):
        pts = []
        for phase in plan['phases']:
            qs = phase['q']
            step = max(1, len(qs) // 30)
            pts += [site_T_of(self.model, self.scratch, qs[i])[:3, 3] for i in range(0, len(qs), step)]
        return pts

    def preview_plan(self, doc, plan):
        """Plays the plan with no physics: what you see is the planned motion, frame by frame."""
        self.open_viewer()
        m, d = self.model, self.data
        T_wp = np.array(doc['part']['T_world_part'])
        T_tcp_part = np.array(plan['T_tcp_part'])
        self.set_ghost(T_wp @ inv(T_tcp_part))
        self.show_path(self._world_path(plan))
        self._set_part_qpos(T_wp)
        d.qpos[:N_ARM] = self.home
        self.fingers.set_qpos(d.qpos, plan['phases'][0]['width'])
        mujoco.mj_forward(m, d)
        self.snapshot('preview_00_start')
        width_now = plan['phases'][0]['width']
        for k, phase in enumerate(plan['phases']):
            self.phase(phase['name'], f"Preview: {phase['label']}")
            attached = phase['attached']
            frames = max(2, int(phase['seconds'] * 30 / self.speed))
            qs = np.array(phase['q'])
            if len(qs) < 2:
                qs = np.vstack([qs, qs])
            width_a, width_b = width_now, phase['width']
            if phase['name'] == 'grip':  # the fingers close on the part
                width_b = plan['widthMm'] if plan['mode'] == 'outside' else plan['phases'][3]['width']
            for f in range(frames):
                s = 0.5 * (1 - np.cos(np.pi * (f + 1) / frames))
                u = s * (len(qs) - 1)
                i = min(int(u), len(qs) - 2)
                d.qpos[:N_ARM] = qs[i] + (qs[i + 1] - qs[i]) * (u - i)
                self.fingers.set_qpos(d.qpos, width_a + (width_b - width_a) * s)
                mujoco.mj_kinematics(m, d)
                if attached:
                    self._set_part_qpos(site_T(m, d, TCP_SITE) @ T_tcp_part)
                d.qvel[:] = 0
                mujoco.mj_forward(m, d)
                if self.viewer is not None:
                    if not self.viewer.is_running():
                        raise KeyboardInterrupt
                    self.viewer.sync()
                    time.sleep(1 / 30)
            width_now = width_b
            if attached and plan['phases'][k + 1]['attached'] is False:
                self._set_part_qpos(np.array(plan['T_wp_placed']))
                mujoco.mj_forward(m, d)
            self.snapshot(f"preview_{k + 1:02d}_{phase['name']}")
        return f"Preview finished: {plan['durationS']:.0f} s of motion, nothing was simulated."

    def idle_preview(self):
        """Keep the final preview frame on screen until the window is closed."""
        if self.viewer is None:
            return
        while self.viewer.is_running():
            self.viewer.sync()
            time.sleep(1 / 30)

    # ---------------- execution (physics) ----------------
    def execute_plan(self, doc, plan):
        st, cfg, fm = self.status, self.cfg, self.fingers
        self.open_viewer()
        self.phase('settling', 'Letting the part settle on the table...')
        self.set_ghost(make_T([0, 0, -5], [1, 0, 0, 0]))
        T_wp = self.settle()
        T_planned = np.array(doc['part']['T_world_part'])
        e_mm, e_deg = pose_error(T_wp, T_planned)
        if e_mm > 1.0 or e_deg > 1.0:
            raise PlanError(f'The part settled {e_mm:.1f} mm / {e_deg:.1f} deg from where the plan assumed. '
                            'Press Calculate Grasp again.')

        T_part_tcp = inv(np.array(plan['T_tcp_part']))
        self.T_part_tcp = T_part_tcp
        T_target = T_wp @ T_part_tcp
        self.set_ghost(T_target, ok=True)
        self.show_path(self._world_path(plan))
        self.phase('preview', 'Green ghost = calculated grasp. Yellow dots = planned TCP path.')
        self.snapshot('01_preview')
        self.step(cfg['preview_seconds'])

        inside = plan['mode'] == 'inside'
        grip_ctrl = GRIP_OPEN if inside else GRIP_CLOSED
        z0, T_tcp_part0, grasp_err, slip, finger_error = None, None, None, (0.0, 0.0), None

        for phase in plan['phases']:
            name = phase['name']
            self.phase(name, phase['label'] + '...')
            qs = phase['q']
            if name in ('pre-grasp', 'approach', 'retreat', 'home'):
                self.move(qs, phase['seconds'], fm.ctrl_for(phase['width']))
            elif name == 'grip':
                self.step(0.4)
                grasp_err = pose_error(inv(self.part_T()) @ site_T(self.model, self.data, TCP_SITE), T_part_tcp)
                st.info['grasp_pose_error'] = {'mm': round(grasp_err[0], 2), 'deg': round(grasp_err[1], 2)}
                self.snapshot('02_at_grasp')
                self.hide_ghost()
                self.move(qs, phase['seconds'], grip_ctrl)
                self.step(0.6)
                T_tcp_part0 = inv(site_T(self.model, self.data, TCP_SITE)) @ self.part_T()
                z0 = self.part_T()[2, 3]
                com_z0 = self._com_height()
                if doc.get('points'):
                    finger_error = self._finger_to_points(doc['points'])
                    st.info['finger_to_point_mm'] = [round(e, 1) for e in finger_error]
            elif name == 'inspect':
                self.move(qs, phase['seconds'], grip_ctrl)
                self.step(self.insp.get('hold_seconds', 0.8))
                if self._com_height() - com_z0 < 0.5 * cfg['lift_height_m']:
                    raise PlanError(f"The part slipped out of the gripper while being shown to the camera ({phase['view']}). "
                                    'Try a firmer grasp, or fewer tilted views in sim_config.json.')
                images = st.info.setdefault('inspection', [])
                fname = f'inspect_{len(images) + 1:02d}.png'
                if self.feed is not None:
                    self.feed.snap(self.data, fname)
                held, _ = pose_error(inv(site_T(self.model, self.data, TCP_SITE)) @ self.part_T(), T_tcp_part0)
                images.append({'label': phase['view'], 'file': fname, 'slip_mm': round(held, 1)})
                self.snapshot(f'03b_inspect_{len(images):02d}')
            elif name == 'release':
                self.move(qs, phase['seconds'], fm.ctrl_for(phase['width']))
                self.step(0.4)
            else:
                self.move(qs, phase['seconds'], grip_ctrl)
                if name == 'lift':
                    self.step(0.3)
                    self.snapshot('03_lifted')
                    rose = self.part_T()[2, 3] - z0
                    if rose < 0.5 * cfg['lift_height_m']:
                        raise PlanError(f"The part was not picked up (rose {rose * 1000:.0f} mm of "
                                        f"{cfg['lift_height_m'] * 1000:.0f} mm). The grip did not hold its "
                                        f"{self.meta['mass_kg'] * 1000:.0f} g: try another grasp, the Hand-E, "
                                        'or raise the grip force in sim_config.json.')
                if name == 'transfer':
                    slip = pose_error(inv(site_T(self.model, self.data, TCP_SITE)) @ self.part_T(), T_tcp_part0)
                    st.info['slip_during_transport'] = {'mm': round(slip[0], 2), 'deg': round(slip[1], 2)}

        self.step(0.8)
        self.snapshot('04_done')
        T_end = self.part_T()
        px, py = cfg['place_xy']
        com_end = T_end[:3, :3] @ np.asarray(self.meta['com_m']) + T_end[:3, 3]
        in_tray = abs(com_end[0] - px) < 0.065 and abs(com_end[1] - py) < 0.05 and self.part_min_z(T_end) < 0.03
        st.info['part_in_tray'] = bool(in_tray)
        summary = (f"{'Part placed in the tray' if in_tray else 'Part was NOT placed in the tray'}. "
                   f"Grasp accuracy: {grasp_err[0]:.1f} mm / {grasp_err[1]:.1f} deg. "
                   f"Slip during transport: {slip[0]:.1f} mm / {slip[1]:.1f} deg.")
        views = len(st.info.get('inspection', []))
        if views:
            summary += f' Camera: {views} images.'
        if plan.get('inspectionSkipped'):
            st.info['inspection_skipped'] = plan['inspectionSkipped']
            summary += f" Views skipped: {len(plan['inspectionSkipped'])}."
        if finger_error:
            summary += (f' Fingers on the marked points: within {finger_error[0]:.1f} mm (point 1) and '
                        f'{finger_error[1]:.1f} mm (point 2).')
        return bool(in_tray), summary

    def _com_height(self):
        """The part's CAD origin can be anywhere on it and swings when the part turns; its centre of mass doesn't."""
        T = self.part_T()
        return float((T[:3, :3] @ np.asarray(self.meta['com_m']) + T[:3, 3])[2])

    def _finger_to_points(self, points_mm):
        """Distance from each marked point to the finger pressing on the part, measured in the simulation: across
        the grip to the contact, and along the finger to the piece of finger that is in contact. (Where a flat finger
        meets a flat wall, the simulator reports a contact point anywhere along the overlap, so that point says little.)"""
        m, d = self.model, self.data
        T_tcp_world = inv(site_T(m, d, TCP_SITE))
        T_tcp_part = T_tcp_world @ self.part_T()
        part = m.body('cad_part').id
        touching = []  # (contact position across the grip, lower bounds, upper bounds of the finger piece)
        for c in d.contact[:d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if part in (b1, b2) and m.body(b2 if b1 == part else b1).name.startswith('g_'):
                geom = c.geom2 if b1 == part else c.geom1
                lo, hi = GripperGeometry._bounds(m, d, geom, T_tcp_world)
                touching.append(((T_tcp_world[:3, :3] @ c.pos + T_tcp_world[:3, 3])[0], lo, hi))
        errors = []
        for point in points_mm:
            p = T_tcp_part[:3, :3] @ (np.asarray(point) / 1000.0) + T_tcp_part[:3, 3]
            best = float('inf')  # stays infinite if no finger touches the part on this side
            for x, lo, hi in touching:
                if np.sign(x) == np.sign(p[0]):
                    along = np.maximum(lo[1:] - p[1:], p[1:] - hi[1:]).clip(min=0)
                    best = min(best, 1000.0 * float(np.linalg.norm([x - p[0], *along])))
            errors.append(best)
        return errors


def site_T_of(model, data, q):
    """TCP pose for an arm posture, using `data` as scratch."""
    saved = data.qpos.copy()
    data.qpos[:N_ARM] = q
    mujoco.mj_kinematics(model, data)
    T = site_T(model, data, TCP_SITE)
    data.qpos[:] = saved
    return T


def parse_vec(text):
    v = np.array([float(x) for x in text.split(',')], dtype=float)
    if v.size != 3:
        raise SystemExit(f'Expected x,y,z, got {text!r}')
    return v


def load_request(args):
    if args.request:
        with open(args.request) as f:
            req = json.load(f)
        step = req['partFile']
    else:
        if not args.step:
            raise SystemExit('Give --request, or --step')
        req, step = {}, args.step
        if args.point1 and args.point2:
            req['points'] = []
            for text in (args.point1, args.point2):
                v = [float(c) for c in text.split(',')]
                if len(v) != 6:
                    raise SystemExit(f'Expected x,y,z,nx,ny,nz, got {text!r}')
                req['points'].append({'point': dict(zip('xyz', v[:3])), 'normal': dict(zip('xyz', v[3:]))})
        if args.click:
            req['click'] = {'point': dict(zip('xyz', parse_vec(args.click))),
                            'normal': dict(zip('xyz', parse_vec(args.normal))) if args.normal else None}
    if args.robot:
        req['robotId'] = args.robot
    if args.gripper:
        req['gripperId'] = args.gripper
    pwp = req.get('partWorldPose')
    q = quat_xyzw_to_wxyz([pwp['orientation'][k] for k in 'xyzw']) if pwp else np.array([1.0, 0, 0, 0])
    return step, req, q


def click_from(req):
    """Clicked point and surface normal in the part frame: mm from the web tool -> m."""
    click = req.get('click') or {}
    p, n = click.get('point'), click.get('normal')
    point = np.array([p['x'], p['y'], p['z']], dtype=float) / 1000.0 if p else None
    normal = None
    if n:
        normal = np.array([n['x'], n['y'], n['z']], dtype=float)
        length = np.linalg.norm(normal)
        normal = normal / length if length > 1e-6 else None
    return point, normal


def points_from(req):
    """The two marked points and surface normals in the part frame: mm from the web tool -> (m, unit), or None."""
    marked = req.get('points')
    if not marked:
        return None
    out = []
    for item in marked:
        p, n = item['point'], item['normal']
        normal = np.array([n['x'], n['y'], n['z']], dtype=float)
        out.append((np.array([p['x'], p['y'], p['z']], dtype=float) / 1000.0, normal / np.linalg.norm(normal)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['plan', 'preview', 'execute'], default='plan')
    ap.add_argument('--request', help='request.json written by the web app bridge')
    ap.add_argument('--status', help='status.json to update (polled by the web app)')
    ap.add_argument('--plan', required=True, help='plan.json: written by --mode plan, read by the other modes')
    ap.add_argument('--candidate', type=int, default=0, help='which of the planned grasps to preview or execute')
    ap.add_argument('--step', help='STEP file (manual mode)')
    ap.add_argument('--click', help='clicked point x,y,z in the part frame, mm (manual mode)')
    ap.add_argument('--normal', help='surface normal at the click, x,y,z (manual mode)')
    ap.add_argument('--point1', help='where one finger goes: x,y,z,nx,ny,nz in the part frame, mm (manual mode)')
    ap.add_argument('--point2', help='where the other finger goes (manual mode)')
    ap.add_argument('--robot', choices=list(ROBOTS))
    ap.add_argument('--gripper', choices=list(GRIPPER_NAMES))
    ap.add_argument('--config', default=os.path.join(BASE_DIR, 'sim_config.json'))
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--snapshots', help='directory for per-phase PNGs (headless)')
    args = ap.parse_args()

    status = Status(args.status)
    sim = None
    try:
        with open(args.config) as f:
            cfg = json.load(f)
        step, req, q_part = load_request(args)
        headless = args.headless or args.mode == 'plan'

        if args.mode == 'plan':
            sim = AutoPickPlace(step, req, q_part, cfg, status, headless=True)
            point, normal = click_from(req)
            doc = sim.compute_plan(point, normal, points=points_from(req))
            tmp = args.plan + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(doc, f)
            os.replace(tmp, args.plan)
            best = doc['plans'][0]
            status.update('done', f"{doc['note']}{len(doc['plans'])} complete motion(s) planned in {doc['seconds']:.0f} s. Best "
                                  f"grasp: {best['mode']} grip, {best['widthMm']:.1f} mm wide, approach "
                                  f"{best['approachTiltDeg']:.0f} deg from vertical.", state='done', ok=True)
            return 0

        with open(args.plan) as f:
            doc = json.load(f)
        if not 0 <= args.candidate < len(doc['plans']):
            raise PlanError(f"The plan has {len(doc['plans'])} grasps; there is no number {args.candidate}.")
        plan = doc['plans'][args.candidate]
        req = dict(req, robotId=doc['robot'], gripperId=doc['gripper'], tcpOffset=doc['tcpOffset'],
                   preGraspDistance=plan['grasp']['preGraspDistance'], grasp=plan['grasp'])
        feed_dir = os.path.dirname(os.path.abspath(args.status)) if args.status and args.mode == 'execute' else None
        sim = AutoPickPlace(step, req, q_part, cfg, status, headless=headless, snapshots=args.snapshots, feed_dir=feed_dir)
        if args.mode == 'preview':
            message = sim.preview_plan(doc, plan)
            status.update('done', message, state='done', ok=True)
            sim.overlay('PREVIEW DONE', 'Close this window, then press Execute in the web app.')
            sim.idle_preview()
            return 0
        ok, summary = sim.execute_plan(doc, plan)
        status.update('done', summary, state='done', ok=ok)
        sim.overlay('DONE' if ok else 'FINISHED (NOT IN TRAY)', summary + '  Close this window when finished.')
        sim.idle_until_closed()
        return 0 if ok else 2
    except PlanError as e:
        status.update('failed', str(e), state='failed', ok=False)
        if sim is not None and sim.viewer is not None:
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
