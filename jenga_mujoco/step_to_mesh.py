#!/usr/bin/env python3
"""
step_to_mesh.py - Reproducible STEP -> MuJoCo mesh conversion.

Outputs (in METERS, in the part's own CAD frame - no recentring, no rotation):
  visual.stl            full-resolution triangulation (visual only)
  hull_XX.stl           convex pieces from CoACD (collision only)
  meta.json             bbox, volume, mass, COM and inertia (steel by default)

The STEP file is read with OpenCascade (OCP) - the same kernel the web viewer
uses (occt-import-js) - with OCCT's default length unit (mm). Every vertex is
multiplied by 0.001, so a grasp pose exported by the web app in the part frame
(mm) is valid here after the same 0.001 scale.

Usage:
  python step_to_mesh.py ../skf_cam_roller_305703_c-2z_0.stp            # -> assets/cad_parts/<hash>/
  python step_to_mesh.py part.stp --out /tmp/part --density 7850 --max-hulls 24
"""
import argparse
import hashlib
import json
import os
import sys

import numpy as np

MM_TO_M = 0.001
STEEL_DENSITY = 7850.0  # kg/m^3

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CACHE = os.path.join(BASE_DIR, 'assets', 'cad_parts')
CONVERTER_VERSION = 1  # bump to invalidate cached conversions


def read_step(path):
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone

    reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError(f'Could not read STEP file: {path}')
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise RuntimeError(f'STEP file has no geometry: {path}')
    return shape


def triangulate(shape, linear_deflection_mm=0.05, angular_deflection=0.25):
    """Returns (vertices_mm Nx3, faces Mx3) for all faces of the shape, outward winding."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location

    _to_face = getattr(TopoDS, 'Face_s', None) or TopoDS.Face  # OCP < 7.8 vs >= 7.8 naming

    BRepMesh_IncrementalMesh(shape, linear_deflection_mm, False, angular_deflection, True)

    verts, faces, offset = [], [], 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = _to_face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                verts.append((p.X(), p.Y(), p.Z()))
            reversed_face = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                t = tri.Triangle(i)
                a, b, c = t.Value(1), t.Value(2), t.Value(3)
                if reversed_face:
                    b, c = c, b
                faces.append((offset + a - 1, offset + b - 1, offset + c - 1))
            offset += tri.NbNodes()
        exp.Next()

    if not faces:
        raise RuntimeError('Triangulation produced no triangles.')
    return np.asarray(verts, dtype=np.float64), np.asarray(faces, dtype=np.int64)


def mass_properties(shape, density):
    """Exact B-rep volume properties (OCCT), converted to SI. Inertia is about the COM, in the part frame."""
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    vol_mm3 = props.Mass()  # with unit density, "mass" == volume
    if vol_mm3 <= 0:
        raise RuntimeError('STEP solid has zero/negative volume (is it a surface model?).')
    com = props.CentreOfMass()
    I = props.MatrixOfInertia()  # about COM, unit density, mm^5
    I_mm5 = np.array([[I.Value(r, c) for c in (1, 2, 3)] for r in (1, 2, 3)])

    vol_m3 = vol_mm3 * 1e-9
    mass = vol_m3 * density
    inertia = I_mm5 * 1e-15 * density  # kg m^2
    return {
        'volume_m3': vol_m3,
        'mass_kg': mass,
        'com_m': [com.X() * MM_TO_M, com.Y() * MM_TO_M, com.Z() * MM_TO_M],
        # MuJoCo fullinertia order: ixx iyy izz ixy ixz iyz
        'fullinertia': [inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2]],
    }


def convex_decompose(verts_m, faces, threshold, max_hulls):
    import coacd
    coacd.set_log_level('error')
    parts = coacd.run_coacd(coacd.Mesh(verts_m, faces), threshold=threshold, max_convex_hull=max_hulls)
    return [(np.asarray(v), np.asarray(f)) for v, f in parts]


def write_stl(path, verts, faces):
    import trimesh
    trimesh.Trimesh(verts, faces, process=False).export(path)


def file_hash(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def convert(step_path, out_dir=None, density=STEEL_DENSITY, max_hulls=24, threshold=0.04, force=False):
    """Converts (or returns the cached conversion of) a STEP file. Returns the meta dict."""
    digest = file_hash(step_path)
    key = f'{digest[:16]}_d{int(density)}_h{max_hulls}_t{threshold}_v{CONVERTER_VERSION}'
    out_dir = out_dir or os.path.join(DEFAULT_CACHE, key)
    meta_path = os.path.join(out_dir, 'meta.json')
    if not force and os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
        if meta.get('key') == key:
            meta['dir'] = out_dir
            return meta

    os.makedirs(out_dir, exist_ok=True)
    shape = read_step(step_path)
    verts_mm, faces = triangulate(shape)
    verts_m = verts_mm * MM_TO_M
    write_stl(os.path.join(out_dir, 'visual.stl'), verts_m, faces)

    hulls = convex_decompose(verts_m, faces, threshold, max_hulls)
    hull_files = []
    for i, (hv, hf) in enumerate(hulls):
        name = f'hull_{i:02d}.stl'
        write_stl(os.path.join(out_dir, name), hv, hf)
        hull_files.append(name)

    mp = mass_properties(shape, density)
    bb_min, bb_max = verts_mm.min(axis=0), verts_mm.max(axis=0)
    meta = {
        'key': key,
        'source': os.path.abspath(step_path),
        'source_sha256': digest,
        'units': 'm (part CAD frame, scaled 0.001 from STEP mm)',
        'density_kg_m3': density,
        'bbox_mm': {'min': bb_min.round(3).tolist(), 'max': bb_max.round(3).tolist(),
                    'size': (bb_max - bb_min).round(3).tolist()},
        'visual_mesh': 'visual.stl',
        'visual_triangles': int(len(faces)),
        'collision_hulls': hull_files,
        **mp,
    }
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)
    meta['dir'] = out_dir
    return meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('step')
    ap.add_argument('--out', default=None)
    ap.add_argument('--density', type=float, default=STEEL_DENSITY)
    ap.add_argument('--max-hulls', type=int, default=24)
    ap.add_argument('--threshold', type=float, default=0.04, help='CoACD concavity threshold (lower = more hulls)')
    ap.add_argument('--force', action='store_true')
    ap.add_argument('--view', action='store_true', help='open a MuJoCo viewer showing hulls over the visual mesh')
    args = ap.parse_args()

    meta = convert(args.step, args.out, args.density, args.max_hulls, args.threshold, args.force)
    s = meta['bbox_mm']['size']
    print(f"Output        : {meta['dir']}")
    print(f"BBox (mm)     : min {meta['bbox_mm']['min']}  max {meta['bbox_mm']['max']}")
    print(f"Size (mm)     : {s[0]:.2f} x {s[1]:.2f} x {s[2]:.2f}   <- compare with the web viewer's Properties panel")
    print(f"Volume        : {meta['volume_m3'] * 1e6:.3f} cm^3")
    print(f"Mass          : {meta['mass_kg'] * 1000:.1f} g  (density {meta['density_kg_m3']:.0f} kg/m^3)")
    print(f"COM (mm)      : {[round(c * 1000, 3) for c in meta['com_m']]}")
    print(f"Visual tris   : {meta['visual_triangles']}")
    print(f"Hulls         : {len(meta['collision_hulls'])}")

    if args.view:
        import mujoco
        import mujoco.viewer
        from cad_part import add_cad_part
        spec = mujoco.MjSpec()
        spec.worldbody.add_light(pos=[0, 0, 1], dir=[0, 0, -1])
        add_cad_part(spec, meta, pos=[0, 0, 0], quat=[1, 0, 0, 0], free=False, hull_alpha=0.5)
        mujoco.viewer.launch(spec.compile())


if __name__ == '__main__':
    sys.exit(main())
