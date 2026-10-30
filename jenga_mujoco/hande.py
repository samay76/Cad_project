"""
hande.py - Robotiq Hand-E for the MuJoCo scene, built from the same URDF/meshes as the web tool
(public/grippers/robotiq_hande, from robotiq_hande_description, Apache-2.0).

Frames match the URDF exactly: body 'base' = robotiq_hande_coupler = tool flange, +Z = approach,
fingers slide along X. So the web TCP is simply (0, 0, tcpOffset) on 'base' (no offset/rotation as
for the Menagerie 2F-85, see frames.py).

  base            coupler (11 mm) + Hand-E body (99 mm); collision = the URDF cylinders
  left_finger     slide joint along +X at z = 110 mm, 0..25 mm (opening = 2 x travel, 0..50 mm)
  right_finger    same, mirrored (body rotated 180 deg about Z), coupled to the left finger
  fingers_actuator  ctrl 0..255 like the 2F-85 in this project: 0 = open (50 mm), 255 = closed;
                  force-limited position control (the grip force is the force limit)

Meshes are converted from the .dae files once (assets/hande/), the finger collision mesh is split
into convex pieces (CoACD) so a stepped finger is not filled in by MuJoCo's convex hull.
"""
import json
import os

import numpy as np
import mujoco

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_MESHES = os.path.join(BASE_DIR, '..', 'public', 'grippers', 'robotiq_hande', 'meshes')
CACHE = os.path.join(BASE_DIR, 'assets', 'hande')
VERSION = 1

COUPLER_H, BODY_H = 0.011, 0.099
TRAVEL = 0.025            # m per finger
MAX_OPENING_MM = 50.0
FINGER_JOINTS = ['left_finger_joint', 'right_finger_joint']
KP, KV = 3000.0, 60.0     # finger position servo
DEFAULT_FORCE_N = 60.0    # Hand-E: 20..185 N grip force


def _load(name):
    import trimesh
    s = trimesh.load(os.path.join(SRC_MESHES, f'{name}.dae'))
    if isinstance(s, trimesh.Scene):
        s = s.to_geometry() if hasattr(s, 'to_geometry') else s.dump(concatenate=True)
    return s


def _assets():
    """STL files (visual meshes, finger collision pieces); converted once and cached."""
    out = os.path.join(CACHE, f'v{VERSION}')
    index = os.path.join(out, 'index.json')
    if os.path.exists(index):
        with open(index) as f:
            idx = json.load(f)
        return {k: (os.path.join(out, v) if isinstance(v, str) else [os.path.join(out, p) for p in v])
                for k, v in idx.items()}
    import coacd
    import trimesh
    os.makedirs(out, exist_ok=True)
    idx = {}
    for name in ('io_coupler', 'hande', 'finger'):
        m = _load(name)
        if len(m.faces) > 40000:  # display only: keep MuJoCo compile fast
            try:
                m = m.simplify_quadric_decimation(face_count=40000)
            except Exception:  # noqa: BLE001  (decimation backend not installed: keep full mesh)
                pass
        m.export(os.path.join(out, f'{name}.stl'))
        idx[name] = f'{name}.stl'
    col = _load('finger_collision')
    coacd.set_log_level('error')
    parts = coacd.run_coacd(coacd.Mesh(np.asarray(col.vertices), np.asarray(col.faces)), threshold=0.02,
                            max_convex_hull=6)
    idx['finger_col'] = []
    for i, (v, f) in enumerate(parts):
        p = f'finger_col_{i:02d}.stl'
        trimesh.Trimesh(np.asarray(v), np.asarray(f), process=False).export(os.path.join(out, p))
        idx['finger_col'].append(p)
    with open(index, 'w') as f:
        json.dump(idx, f)
    return _assets()


def build_spec():
    """MjSpec of the Hand-E (attach to the arm with prefix 'g_')."""
    a = _assets()
    s = mujoco.MjSpec()
    s.compiler.degree = False
    for n in ('io_coupler', 'hande', 'finger'):
        s.add_mesh(name=n, file=a[n])
    for i, p in enumerate(a['finger_col']):
        s.add_mesh(name=f'finger_col_{i:02d}', file=p)

    vis = dict(contype=0, conaffinity=0, group=2, density=0)
    col = dict(group=3, density=0, rgba=[0.6, 0.3, 0.3, 0.5])

    base = s.worldbody.add_body(name='base')
    base.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname='io_coupler', rgba=[0.15, 0.15, 0.15, 1], **vis)
    base.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname='hande', pos=[0, 0, COUPLER_H],
                  rgba=[0.12, 0.12, 0.13, 1], **vis)
    # URDF collision cylinders (coupler shell + body)
    base.add_geom(name='coupler_col', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.0385, 0.0169 / 2],
                  pos=[0, 0, 0.0169 / 2 - 0.003], **col)
    base.add_geom(name='body_col', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.0385, (BODY_H + 0.0029 + 0.005) / 2],
                  pos=[0, 0, COUPLER_H + (BODY_H + 0.0029 + 0.005) / 2], **col)
    # coupler 0.119 kg + Hand-E 0.864 kg (cylinder inertia as in the URDF)
    base.mass, base.ipos = 0.983, [0, 0, 0.06]
    base.inertia = [0.0016, 0.0016, 0.0007]
    base.explicitinertial = True

    for side, quat in (('left', [1, 0, 0, 0]), ('right', [0, 0, 0, 1])):
        f = base.add_body(name=f'{side}_finger', pos=[0, 0, COUPLER_H + BODY_H], quat=quat)
        f.mass, f.inertia, f.explicitinertial = 0.038, [1e-5, 1e-5, 1e-5], True
        f.add_joint(name=f'{side}_finger_joint', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1, 0, 0],
                    range=[0, TRAVEL], limited=mujoco.mjtLimited.mjLIMITED_TRUE, armature=0.01, damping=2.0)
        f.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname='finger', rgba=[0.55, 0.57, 0.6, 1], **vis)
        for i in range(len(a['finger_col'])):
            f.add_geom(name=f'{side}_pad_{i:02d}', type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f'finger_col_{i:02d}',
                       friction=[1.0, 0.005, 0.0001], priority=1, condim=4, solimp=[0.95, 0.99, 0.001, 0.5, 2],
                       solref=[0.004, 1], **col)

    # The URDF body cylinder overlaps the finger roots by ~2 mm: never collide fingers with their own body
    for side in ('left', 'right'):
        s.add_exclude(bodyname1='base', bodyname2=f'{side}_finger')
    s.add_equality(type=mujoco.mjtEq.mjEQ_JOINT, name1='right_finger_joint', name2='left_finger_joint',
                   data=np.r_[0, 1, 0, 0, 0, np.zeros(7)], solimp=[0.95, 0.99, 0.001, 0.5, 2])
    # force = KP * (target - q) - KV * qdot, target = TRAVEL * (1 - ctrl / 255)
    act = s.add_actuator(name='fingers_actuator', target='left_finger_joint', trntype=mujoco.mjtTrn.mjTRN_JOINT,
                         gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                         ctrlrange=[0, 255], ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
                         forcerange=[-DEFAULT_FORCE_N, DEFAULT_FORCE_N],
                         forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE)
    act.gainprm[0] = -KP * TRAVEL / 255.0
    act.biasprm[:3] = [KP * TRAVEL, -KP, -KV]
    return s


if __name__ == '__main__':
    import mujoco.viewer
    m = build_spec().compile()
    print('Hand-E: bodies', m.nbody, 'geoms', m.ngeom, 'joints', m.njnt, 'actuators', m.nu)
    mujoco.viewer.launch(m)
