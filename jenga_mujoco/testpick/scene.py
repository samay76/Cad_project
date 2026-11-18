"""
scene.py - The testpick MuJoCo scene: the main project's world, arm, gripper and part, plus a scan camera.

Everything except the camera rig and the attach weld comes from the existing project, called the same way
cad_pick_place.py calls it (cad_pick_place.py:262-310):

  build_robot.build()        floor + Table_base table + tray + light (world.add_world / table_base.add_table),
                             EC66 on its mounting plate (+ robot_base_offset_mm), 2F-85 attached with prefix g_
                             (+ gripper_coupling_mm), g_tcp site, the CAD part on a free joint (cad_part.py)
  step_to_mesh.convert()     STEP -> visual mesh + CoACD hulls (cached here in testpick/cache/)
  sim_config.json            pick_xy, pick_station, coupling, base offset, grip force, home joints

The `extra` hook of build() adds what testpick needs on top, without touching the original files:
  - scan_cam_rig: a camera-shaped body (dark box + lens + post + foot, all contype=conaffinity=0) with the
    <camera name="scan_cam"> at the lens, looking along its -Z at the scan point
  - testpick_attach: a weld equality g_base <-> cad_part, inactive until the gripper has closed
"""
import json
import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)  # jenga_mujoco/
if PROJECT not in sys.path:
    sys.path.insert(0, PROJECT)

from build_robot import build                                        # noqa: E402
from cad_part import PART_BODY, PART_JOINT                           # noqa: E402
from cad_pick_place import (robot_base_offset, tray_half_for, finger_map, TCP_SITE,  # noqa: E402
                            GRIPPER_NAMES)
from cameras import lookat_quat                                      # noqa: E402
from frames import make_T, site_T                                    # noqa: E402
from ik import solve as ik_solve, N_ARM                              # noqa: E402
from pick_station import from_config as station_from_config          # noqa: E402
from robots import get_robot                                         # noqa: E402
from step_to_mesh import convert, file_hash, is_mesh_file            # noqa: E402

CACHE_DIR = os.path.join(HERE, 'cache')
SIM_CONFIG = os.path.join(PROJECT, 'sim_config.json')
CAM_BODY = 'scan_cam_rig'
WELD_NAME = 'testpick_attach'
PART_GEOM = f'{PART_BODY}_visual'
G = mujoco.mjtGeom


def load_configs(config_path):
    with open(config_path) as f:
        cfg = json.load(f)
    with open(SIM_CONFIG) as f:
        sim_cfg = json.load(f)
    return cfg, sim_cfg


def convert_part(step_path, sim_cfg):
    """STEP/STL -> meshes with the project's converter, cached under testpick/cache/<file hash>_d<density>."""
    density = sim_cfg.get('mesh_density_kg_m3', 2700) if is_mesh_file(step_path) else sim_cfg.get('density_kg_m3', 7850)
    out_dir = os.path.join(CACHE_DIR, f'{file_hash(step_path)[:16]}_d{int(density)}')
    return convert(step_path, out_dir=out_dir, density=density)


def camera_pose(cfg):
    """Camera position from the scan point, distance, elevation and azimuth (config.json 'camera')."""
    c = cfg['camera']
    el, az = np.radians(c['elevation_deg']), np.radians(c['azimuth_deg'])
    direction = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    scan = np.asarray(cfg['scan_point'], dtype=float)
    return scan + c['distance_m'] * direction, scan


def add_scan_camera(spec, eye, target, fovy, name):
    """Camera-shaped rig on a post standing on the table. No collisions: it is a stand-in for a sensor."""
    q_cam = lookat_quat(eye, target)
    R = make_T([0, 0, 0], q_cam)[:3, :3]
    body = spec.worldbody.add_body(name=CAM_BODY, pos=eye.tolist())
    cam = body.add_camera(name=name, pos=[0, 0, 0], quat=q_cam.tolist())
    cam.fovy = float(fovy)
    vis = dict(contype=0, conaffinity=0, group=0)
    # Dark housing behind the optical centre (camera +Z = backwards)
    body.add_geom(name=f'{name}_housing', type=G.mjGEOM_BOX, size=[0.04, 0.025, 0.03],
                  pos=(R @ [0, 0, 0.032]).tolist(), quat=q_cam.tolist(), rgba=[0.12, 0.12, 0.14, 1], **vis)
    # Lens barrel sticking out towards the target, with a blue glass disc
    body.add_geom(name=f'{name}_lens', type=G.mjGEOM_CYLINDER, size=[0.014, 0.006],
                  pos=(R @ [0, 0, -0.004]).tolist(), quat=q_cam.tolist(), rgba=[0.05, 0.05, 0.06, 1], **vis)
    body.add_geom(name=f'{name}_glass', type=G.mjGEOM_CYLINDER, size=[0.010, 0.0008],
                  pos=(R @ [0, 0, -0.0105]).tolist(), quat=q_cam.tolist(), rgba=[0.15, 0.35, 0.75, 1], **vis)
    # Vertical post from the table top (z = 0) up to the housing, and a round foot
    top = float(eye[2]) - 0.035
    body.add_geom(name=f'{name}_post', type=G.mjGEOM_CYLINDER, size=[0.009, top / 2],
                  pos=[0, 0, -0.035 - top / 2], rgba=[0.25, 0.25, 0.27, 1], **vis)
    body.add_geom(name=f'{name}_foot', type=G.mjGEOM_CYLINDER, size=[0.04, 0.004],
                  pos=[0, 0, -float(eye[2]) + 0.004], rgba=[0.2, 0.2, 0.22, 1], **vis)


def add_attach_weld(spec):
    """Weld g_base <-> part, inactive at start. Its relpose is set from the actual poses when it is switched on."""
    eq = spec.add_equality(name=WELD_NAME, type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                           name1='g_base', name2=PART_BODY)
    eq.active = False


class Scene:
    """Model + the ids and settings every other module needs."""

    def __init__(self, step_path, cfg, sim_cfg):
        self.cfg, self.sim_cfg = cfg, sim_cfg
        self.robot = get_robot(cfg.get('robot', 'ec66'))
        self.gripper = cfg.get('gripper', '2f85')
        if self.gripper != '2f85':
            raise SystemExit('testpick is written for the Robotiq 2F-85 (config.json "gripper": "2f85").')
        print(f'[scene] converting {step_path} (cached in {CACHE_DIR})...', flush=True)
        self.meta = meta = convert_part(step_path, sim_cfg)
        print(f"[scene] part: {meta['bbox_mm']['size']} mm, {meta['mass_kg'] * 1000:.0f} g, "
              f"{meta['visual_triangles']} triangles, {len(meta['collision_hulls'])} hulls", flush=True)

        # --- part placement on the table: same logic as cad_pick_place.py:262-283 (partWorldPose = identity) ---
        import trimesh
        verts = np.asarray(trimesh.load(os.path.join(meta['dir'], meta['visual_mesh']), force='mesh').vertices)
        part_quat = np.array([1.0, 0.0, 0.0, 0.0])
        R = make_T([0, 0, 0], part_quat)[:3, :3]
        rv = verts @ R.T
        c = (rv.min(0) + rv.max(0)) / 2
        sim_cfg = dict(sim_cfg)
        station = station_from_config(sim_cfg)
        self.support_z = 0.0
        if station[1] is not None:
            sim_cfg['pick_xy'] = station[1]['pick_xy']
            self.support_z = station[1]['support_z']
            c = R @ np.asarray(meta['com_m'])
        px, py = sim_cfg['pick_xy']
        part_pos = [px - c[0], py - c[1], self.support_z - rv.min(0)[2] + 0.002]
        tray_half = tray_half_for(rv, R @ np.asarray(meta['com_m']), sim_cfg)

        eye, target = camera_pose(cfg)
        self.cam_eye, self.scan_point = eye, target
        cam_name = cfg['camera']['name']

        def extra(spec):
            add_scan_camera(spec, eye, target, cfg['camera']['fovy_deg'], cam_name)
            add_attach_weld(spec)

        coupling = float((sim_cfg.get('gripper_coupling_mm') or {}).get(self.robot.id, {}).get(self.gripper, 0.0))
        self.model = m = build(
            obj='cad_part', cad_meta=meta, part_pos=part_pos, part_quat=part_quat, tray_half=tray_half,
            tcp_offset_mm=float(cfg.get('tcp_offset_mm', 145.0)), gravcomp=True, fine_fingers=True,
            gripper=self.gripper, include_sensor_cameras=False, robot=self.robot.id,
            pick_station=station if station[1] is not None else None, coupling_mm=coupling,
            base_offset_mm=robot_base_offset(sim_cfg, self.robot.id), extra=extra)

        # Same grip force limit as cad_pick_place.py:315-318
        self.grip_act = m.actuator('g_fingers_actuator').id
        f = float(sim_cfg.get('grip_force_limit', 5.0))
        m.actuator_forcerange[self.grip_act] = [-f, f]

        self.part_bid = m.body(PART_BODY).id
        self.part_geom = m.geom(PART_GEOM).id
        self.part_qadr = int(m.jnt_qposadr[m.joint(PART_JOINT).id])
        self.part_dadr = int(m.jnt_dofadr[m.joint(PART_JOINT).id])
        self.cam_id = m.camera(cam_name).id
        self.cam_body = m.body(CAM_BODY).id
        self.weld_id = m.equality(WELD_NAME).id
        self.gbase_bid = m.body('g_base').id
        self.scratch = mujoco.MjData(m)
        self.home = self._home(sim_cfg)
        self.fingers = finger_map(m, self.gripper, self.home)
        print(f'[scene] {self.robot.label} + {GRIPPER_NAMES[self.gripper]}; scan camera at '
              f'{np.round(eye, 3).tolist()} looking at {np.round(target, 3).tolist()}', flush=True)

    def _home(self, sim_cfg):
        """Home posture as cad_pick_place._central_home: taught joints if configured, else TCP down at home_tcp_m."""
        m, s = self.model, self.scratch
        pick = sim_cfg.get('pick_motion') or {}
        fixed = (pick.get('home_joints_deg') or {}).get(self.robot.id)
        if fixed:
            q = np.radians(np.asarray(fixed, dtype=float))
        else:
            s.qpos[:N_ARM] = self.robot.home_qpos
            mujoco.mj_kinematics(m, s)
            T0 = site_T(m, s, TCP_SITE)
            z = np.array([0.0, 0.0, -1.0])
            x = T0[:3, 0] - (T0[:3, 0] @ z) * z
            x /= np.linalg.norm(x)
            T = np.eye(4)
            T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
            T[:3, 3] = pick.get('home_tcp_m', [0.40, 0.0, 0.40])
            r = ik_solve(m, s, TCP_SITE, T, [self.robot.home_qpos])
            q = r.q if r.ok else self.robot.home_qpos
        s.qpos[:N_ARM] = q
        mujoco.mj_kinematics(m, s)
        self.T_home = site_T(m, s, TCP_SITE)
        return q

    # ---------------- attach weld ----------------
    def attach(self, d):
        """Switch the weld on with relpose = the part's CURRENT pose relative to g_base (no jump)."""
        m = self.model
        R1 = d.xmat[self.gbase_bid].reshape(3, 3)
        rel_pos = R1.T @ (d.xpos[self.part_bid] - d.xpos[self.gbase_bid])
        q1inv, rel_quat = np.zeros(4), np.zeros(4)
        mujoco.mju_negQuat(q1inv, d.xquat[self.gbase_bid])
        mujoco.mju_mulQuat(rel_quat, q1inv, d.xquat[self.part_bid])
        m.eq_data[self.weld_id][:] = 0.0
        m.eq_data[self.weld_id][3:6] = rel_pos
        m.eq_data[self.weld_id][6:10] = rel_quat
        m.eq_data[self.weld_id][10] = 1.0  # torquescale
        d.eq_active[self.weld_id] = 1

    def detach(self, d):
        d.eq_active[self.weld_id] = 0
