#!/usr/bin/env python3
"""
table_base.py - The robot table from Table_base.step (Onshape export, repo root).

The STEP is a cabinet with an 800 x 800 mm perforated top, a 200 x 200 x 15 mm mounting plate for the
arm on that top (off-centre, near one edge), and a teach-pendant holder hanging off the opposite edge.

World frame (unchanged for the rest of the sim): table top surface at z = 0, robot base axis at x = y = 0,
the arm looks along +x over the free part of the table. The robot base sits on top of the mounting plate,
i.e. at z = MOUNT_PLATE_TOP_Z (build_robot.py raises the arm by that much).

  CAD (mm, Y up)  ->  world (m, Z up):   x_w = -(x - MOUNT_X),  y_w = -(z - MOUNT_Z),  z_w = -y - TOP_Y

Physics uses simple boxes (table top, mounting plate, pendant). The STEP mesh is visual only: a convex
collision hull of a whole cabinet would fill the space under the top.

The STEP is triangulated once (~15 s, needs OCP like step_to_mesh.py) and cached in var/cache/table_base/.
  python -m cell.table_base            # (re)build the cache
"""
import glob
import hashlib
import json
import os
import sys

import mujoco
import numpy as np
from .paths import CACHE, MODELS

G = mujoco.mjtGeom
TABLE_STEP = os.path.join(MODELS, 'Table_base.step')
CACHE_DIR = os.path.join(CACHE, 'table_base')
CONVERTER_VERSION = 1

# Measured from Table_base.step (mm, CAD frame, Y up)
MOUNT_X, MOUNT_Z = 224.6, -25.2   # centre of the 200 x 200 mm arm mounting plate
TOP_Y = -38.0                     # table top surface (the CAD's "up" is -Y)

# Derived world-frame dimensions (m)
MOUNT_PLATE_HALF = 0.100
MOUNT_PLATE_TOP_Z = 0.015         # robot base height above the table top
TOP_HALF = 0.400                  # 800 x 800 mm top
TOP_THICK = 0.008
TOP_CENTER = (MOUNT_X / 1000.0, MOUNT_Z / 1000.0)   # = (0.2246, -0.0252): x from -0.175 to 0.625
FLOOR_Z = -0.852                  # bottom of the feet
# Screw holes (6.8 mm) of the perforated top: a 40 mm grid, world frame (m). The pick station bolts onto it.
HOLE_GRID_PITCH = 0.040
HOLE_GRID_ORIGIN = (-0.1204, -0.3602)   # hole with the lowest x and y
HOLE_GRID_MAX = (0.5596, 0.3198)        # hole with the highest x and y
# Teach pendant in its holder, beyond the +x edge
PENDANT_MIN = (0.686, -0.486, 0.0)
PENDANT_MAX = (0.845, -0.193, 0.302)

MAX_FACES_PER_STL = 150000        # MuJoCo's STL reader caps a file at 200k faces
_SKIP_SHELL_MM = 3000.0           # the export holds one broken shell with a ~200 m bounding box


def cad_to_world(v_mm):
    v_mm = np.asarray(v_mm, dtype=np.float64)
    return np.c_[-(v_mm[:, 0] - MOUNT_X), -(v_mm[:, 2] - MOUNT_Z), -v_mm[:, 1] + TOP_Y] * 0.001


def _cache_key(path):
    st = os.stat(path)
    return hashlib.sha1(f'{os.path.abspath(path)}|{st.st_size}|{st.st_mtime_ns}|{CONVERTER_VERSION}'
                        .encode()).hexdigest()[:16]


def convert(step_path=TABLE_STEP, out_dir=CACHE_DIR):
    """Triangulates the STEP into world-frame binary STLs (m). Returns their paths."""
    import trimesh
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.TopAbs import TopAbs_SHELL
    from OCP.TopExp import TopExp_Explorer
    from .step_to_mesh import read_step, triangulate

    shape = read_step(step_path)
    verts, faces, offset = [], [], 0
    exp = TopExp_Explorer(shape, TopAbs_SHELL)
    while exp.More():
        shell = exp.Current()
        exp.Next()
        box = Bnd_Box()
        BRepBndLib.Add_s(shell, box)
        lo, hi = box.CornerMin(), box.CornerMax()
        if max(hi.X() - lo.X(), hi.Y() - lo.Y(), hi.Z() - lo.Z()) > _SKIP_SHELL_MM:
            continue
        try:
            v, f = triangulate(shell, linear_deflection_mm=1.0, angular_deflection=0.5)
        except RuntimeError:
            continue
        verts.append(v)
        faces.append(f + offset)
        offset += len(v)
    if not faces:
        raise RuntimeError(f'No geometry in {step_path}')
    v_w = cad_to_world(np.vstack(verts))
    f_all = np.vstack(faces)

    os.makedirs(out_dir, exist_ok=True)
    for old in glob.glob(os.path.join(out_dir, 'table_*.stl')):
        os.remove(old)
    paths = []
    for i, start in enumerate(range(0, len(f_all), MAX_FACES_PER_STL)):
        mesh = trimesh.Trimesh(v_w, f_all[start:start + MAX_FACES_PER_STL], process=False)
        mesh.remove_unreferenced_vertices()
        p = os.path.join(out_dir, f'table_{i:02d}.stl')
        mesh.export(p)
        paths.append(p)
    with open(os.path.join(out_dir, 'meta.json'), 'w') as fh:
        json.dump({'key': _cache_key(step_path), 'files': [os.path.basename(p) for p in paths],
                   'faces': int(len(f_all)), 'bbox_m': [v_w.min(0).tolist(), v_w.max(0).tolist()]}, fh, indent=1)
    return paths


def mesh_files(step_path=TABLE_STEP, out_dir=CACHE_DIR):
    """Cached table STLs, converting the STEP when it is new or changed. [] when it cannot be loaded."""
    meta_path = os.path.join(out_dir, 'meta.json')
    try:
        with open(meta_path) as fh:
            meta = json.load(fh)
        files = [os.path.join(out_dir, n) for n in meta['files']]
        if (not os.path.exists(step_path) or meta['key'] == _cache_key(step_path)) and \
                all(os.path.exists(p) for p in files):
            return files
    except (OSError, ValueError, KeyError):
        pass
    if not os.path.exists(step_path):
        print(f'[table_base] {step_path} not found: table drawn as plain boxes', file=sys.stderr)
        return []
    try:
        print('[table_base] converting Table_base.step (one-time, ~15 s)...', file=sys.stderr)
        if step_path == TABLE_STEP and out_dir == CACHE_DIR:
            # In a clean process: OpenCascade crashes on this big STEP once MuJoCo & co. are loaded in the same one.
            import subprocess
            r = subprocess.run([sys.executable, '-m', 'cell.table_base'], cwd=os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), capture_output=True, text=True, timeout=600)
            if r.returncode != 0:
                raise RuntimeError(f'converter exited with {r.returncode}')
            return mesh_files(step_path, out_dir) if os.path.exists(meta_path) else []
        return convert(step_path, out_dir)
    except Exception as e:  # missing OCP/trimesh, unreadable file: the sim still runs on the boxes
        print(f'[table_base] could not convert the table STEP ({e}): table drawn as plain boxes', file=sys.stderr)
        return []


def add_table(spec):
    """Adds the table (visual mesh + collision boxes) and the floor to spec's worldbody."""
    w = spec.worldbody
    files = mesh_files()
    visual = bool(files)

    w.add_geom(name='floor', type=G.mjGEOM_PLANE, size=[3, 3, 0.05],
               pos=[0, 0, FLOOR_Z], rgba=[0.55, 0.55, 0.55, 1])

    for i, path in enumerate(files):
        spec.add_mesh(name=f'table_base_{i:02d}', file=path)
        w.add_geom(name=f'table_base_visual_{i:02d}', type=G.mjGEOM_MESH, meshname=f'table_base_{i:02d}',
                   rgba=[0.72, 0.73, 0.76, 1], contype=0, conaffinity=0, group=0)

    # Collision boxes. Hidden (group 3) under the mesh; drawn as plain boxes when the mesh is missing.
    hidden = dict(group=3, rgba=[0.75, 0.6, 0.45, 0.0]) if visual else dict(rgba=[0.72, 0.73, 0.76, 1])
    # Kept named 'table_top': the planner and the web preview look for it.
    w.add_geom(name='table_top', type=G.mjGEOM_BOX, size=[TOP_HALF, TOP_HALF, TOP_THICK / 2],
               pos=[TOP_CENTER[0], TOP_CENTER[1], -TOP_THICK / 2], friction=[1.0, 0.005, 0.0001], **hidden)
    w.add_geom(name='table_mount_plate', type=G.mjGEOM_BOX,
               size=[MOUNT_PLATE_HALF, MOUNT_PLATE_HALF, MOUNT_PLATE_TOP_Z / 2],
               pos=[0, 0, MOUNT_PLATE_TOP_Z / 2], **hidden)
    lo, hi = np.array(PENDANT_MIN), np.array(PENDANT_MAX)
    w.add_geom(name='table_pendant', type=G.mjGEOM_BOX, size=((hi - lo) / 2).tolist(),
               pos=((hi + lo) / 2).tolist(), **hidden)
    if not visual:
        h = (-FLOOR_Z - TOP_THICK) / 2
        for i, (sx, sy) in enumerate([(-1, -1), (-1, 1), (1, -1), (1, 1)]):
            w.add_geom(name=f'leg{i}', type=G.mjGEOM_BOX, size=[0.02, 0.02, h],
                       pos=[TOP_CENTER[0] + sx * (TOP_HALF - 0.02), TOP_CENTER[1] + sy * (TOP_HALF - 0.02),
                            -TOP_THICK - h], rgba=[0.4, 0.4, 0.42, 1])


if __name__ == '__main__':
    for p in convert():
        print(p)
