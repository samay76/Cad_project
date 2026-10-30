"""
finger_collision.py - Tighter collision shapes for the Robotiq 2F-85 fingers (CAD scene only).

MuJoCo collides every mesh as its convex hull. For the finger 'tongue' (pad holder) and 'follower'
meshes that hull fills in the thin fingertip: closed fingers measure ~38 mm across in the simulation
although the real fingertips are ~23 mm. That blocks grasps the web tool (exact triangle collision)
accepts, e.g. fingers inserted into a 36 mm bore for an inside grip.

refine(grip_spec) replaces these collision geoms by a CoACD convex decomposition of the same mesh
(same body, pose and contact parameters). Visual geoms are untouched. Pieces are cached in
assets/gripper_hulls/. The Jenga scene does not call this, so its dynamics stay as recorded.
"""
import json
import os

import numpy as np
import mujoco

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(BASE_DIR, 'robotiq_2f85', 'assets')
CACHE_DIR = os.path.join(BASE_DIR, 'assets', 'gripper_hulls')
REFINE = ('tongue', 'follower')
THRESHOLD = 0.05
MAX_PIECES = 8
VERSION = 2


def _pieces(mesh_name):
    """Paths of the convex pieces of robotiq_2f85/assets/<mesh_name>.stl (computed once, then cached)."""
    out_dir = os.path.join(CACHE_DIR, f'{mesh_name}_t{THRESHOLD}_h{MAX_PIECES}_v{VERSION}')
    index = os.path.join(out_dir, 'pieces.json')
    if os.path.exists(index):
        with open(index) as f:
            return [os.path.join(out_dir, p) for p in json.load(f)]
    import coacd
    import trimesh
    src = trimesh.load(os.path.join(MESH_DIR, f'{mesh_name}.stl'), force='mesh')
    coacd.set_log_level('error')
    parts = coacd.run_coacd(coacd.Mesh(np.asarray(src.vertices), np.asarray(src.faces)), threshold=THRESHOLD,
                            max_convex_hull=MAX_PIECES)
    os.makedirs(out_dir, exist_ok=True)
    names = []
    for i, (v, f) in enumerate(parts):
        name = f'{mesh_name}_{i:02d}.stl'
        trimesh.Trimesh(np.asarray(v), np.asarray(f), process=False).export(os.path.join(out_dir, name))
        names.append(name)
    with open(index, 'w') as f:
        json.dump(names, f)
    return [os.path.join(out_dir, n) for n in names]


def refine(grip):
    """Replace the convex-hull collision geoms of the finger meshes in the 2F-85 MjSpec (before attach)."""
    pieces = {n: _pieces(n) for n in REFINE}
    for n, paths in pieces.items():
        for i, p in enumerate(paths):
            grip.add_mesh(name=f'{n}_col_{i:02d}', file=p)
    targets = [g for g in grip.geoms if g.meshname in REFINE and g.contype != 0]
    for g in targets:
        body = g.parent
        for i in range(len(pieces[g.meshname])):
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f'{g.meshname}_col_{i:02d}',
                          pos=g.pos.copy(), quat=g.quat.copy(), contype=g.contype, conaffinity=g.conaffinity,
                          group=g.group, condim=g.condim, friction=g.friction.copy(), solimp=g.solimp.copy(),
                          solref=g.solref.copy(), priority=g.priority, margin=g.margin, gap=g.gap,
                          rgba=g.rgba.copy(), density=0)  # body inertia is explicit; pieces carry no mass
        grip.delete(g)
    return grip
