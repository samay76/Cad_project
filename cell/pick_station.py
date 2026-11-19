#!/usr/bin/env python3
"""
pick_station.py - The pick fixture (cad/Fixture.step) the part is picked from.

The station is a pedestal: a base plate with screw holes and a vertical column. It is bolted to the
table: its screw holes are put on the table's hole grid (table_base.HOLE_GRID_*), on the grid position
nearest pick_station.near_xy in config/cell.json. The part is set on the column's top face, centred on
the column axis, and everything that used "the part on the table at pick_xy" now uses that point:
CadPickPlace replaces cfg['pick_xy'] with the column axis and plans from the top face (support_z).

Fixture mode (pick_station.seated_part in config/cell.json): a part made for this fixture (e.g. cad/
main_part.step on cad/Fixture.step) is put on it exactly as the two STEPs fit together: both are drawn in
the same CAD frame, so the part keeps its STEP coordinates relative to the fixture's (seated_pose). Its CoACD
hulls fill the socket that slides over the fixture top, so instead of resting on contacts the part is held by a
weld (FIXTURE_WELD) until the gripper has closed on it (release_weld), and part <-> fixture contacts are off.
Any other part still sits on the top face as before.

The geometry (holes, column, top face) is read from the STEP once and cached in var/cache/pick_station/.
  python -m cell.pick_station      # prints where the station goes and how well its holes match the table's
"""
import glob
import hashlib
import json
import os
import sys

import mujoco
import numpy as np

from .paths import CACHE, CAD, repo_path, load_config
from .table_base import HOLE_GRID_ORIGIN, HOLE_GRID_PITCH, HOLE_GRID_MAX

G = mujoco.mjtGeom
DEFAULT_STEP = os.path.join(CAD, 'Fixture.step')
CACHE_DIR = os.path.join(CACHE, 'pick_station')
CONVERTER_VERSION = 3          # 2: + top disc; 3: fixtures without a column (anchor = plate centre)
MAX_HOLE_ERROR_MM = 1.0       # a station hole further than this from a table hole does not fit a screw
STATION_BODY = 'pickstation'  # fixture mode: the station geoms are on this static body (for the contact exclude)
FIXTURE_WELD = 'fixture_hold'  # fixture mode: weld part <-> station, released when the gripper has closed


def _cache_key(path):
    st = os.stat(path)
    return hashlib.sha1(f'{os.path.abspath(path)}|{st.st_size}|{st.st_mtime_ns}|{CONVERTER_VERSION}'
                        .encode()).hexdigest()[:16]


def analyse(step_path):
    """Screw holes, column and plate of the station (station frame, mm, Z up, base on z = 0)."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepBndLib import BRepBndLib
    from OCP.Bnd import Bnd_Box
    from OCP.GeomAbs import GeomAbs_Cylinder
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from .step_to_mesh import read_step, triangulate

    shape = read_step(step_path)
    to_face = getattr(TopoDS, 'Face_s', None) or TopoDS.Face

    def bbox(s):
        b = Bnd_Box()
        BRepBndLib.Add_s(s, b)
        p, q = b.CornerMin(), b.CornerMax()
        return np.array([p.X(), p.Y(), p.Z()]), np.array([q.X(), q.Y(), q.Z()])

    lo, hi = bbox(shape)
    vertical = []   # (radius, x, y, zmin, zmax, xy-min, xy-max) of every vertical cylindrical face
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = to_face(exp.Current())
        exp.Next()
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() != GeomAbs_Cylinder:
            continue
        cyl = surf.Cylinder()
        d, loc = cyl.Axis().Direction(), cyl.Axis().Location()
        if abs(abs(d.Z()) - 1.0) > 1e-3:
            continue
        flo, fhi = bbox(face)
        vertical.append((cyl.Radius(), loc.X(), loc.Y(), flo[2], fhi[2], flo[:2], fhi[:2]))
    if not vertical:
        raise RuntimeError(f'{step_path}: no vertical cylinders (column / screw holes) found')

    height = hi[2] - lo[2]
    # Screw holes: full circles (possibly split into several faces) through the base plate: they start at the bottom
    # face and are short. The rounded plate corners are quarter cylinders of a similar radius, so they are told
    # apart by how much of the circle they cover.
    arcs = {}
    for r, x, y, z0, z1, a, b in vertical:
        if z0 <= lo[2] + 0.01 and z1 - z0 < 0.5 * height:
            k = (round(x, 2), round(y, 2), round(r, 2))
            a0, b0 = arcs.get(k, (a, b))
            arcs[k] = (np.minimum(a0, a), np.maximum(b0, b))
    sizes = {}
    for (x, y, r), (a, b) in arcs.items():
        if np.all(b - a >= 1.9 * r):
            sizes.setdefault(r, []).append((x, y))
    if not sizes:
        raise RuntimeError(f'{step_path}: no screw holes found in the base plate')
    hole_r, hole_xy = max(sizes.items(), key=lambda kv: (len(kv[1]), kv[0]))
    if len(hole_xy) < 2:
        raise RuntimeError(f'{step_path}: fewer than 2 screw holes found in the base plate')
    plate_top = min(z1 for r, x, y, z0, z1, a, b in vertical
                    if z0 <= lo[2] + 0.01 and abs(r - hole_r) < 1e-3 and (round(x, 2), round(y, 2)) in
                    {(round(px, 2), round(py, 2)) for px, py in hole_xy})

    # Column (optional): the tallest vertical cylinder above the plate, wider than a screw hole. Fixtures without
    # one (a plate with an upright, e.g. a scale) are anchored at the plate centre and collide with their own
    # convex pieces (add_to_spec).
    cols = [c for c in vertical if c[0] > hole_r + 0.5 and c[4] - c[3] > 0.3 * height]
    column = max(cols, key=lambda c: (c[4] - c[3], c[0])) if cols else None

    # Top disc: a vertical cylinder wider than the column that ends at the top face (a fixture's locating
    # spigot). None for a plain column.
    top = None
    if column is not None:
        tops = [c for c in vertical if c is not column and abs(c[4] - hi[2]) < 0.01 and c[0] > column[0] + 0.5
                and abs(c[1] - column[1]) < 0.5 and abs(c[2] - column[2]) < 0.5]
        top = max(tops, key=lambda c: c[0]) if tops else None
    anchor = [column[1], column[2]] if column is not None else [float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2)]

    v, f = triangulate(shape, linear_deflection_mm=0.1, angular_deflection=0.3)
    return {
        'top_disc_mm': None if top is None else {'radius': top[0], 'z0': top[3]},
        'holes_mm': [list(p) for p in sorted(hole_xy)], 'hole_radius_mm': hole_r,
        'column_xy_mm': None if column is None else [column[1], column[2]],
        'column_radius_mm': None if column is None else column[0], 'anchor_xy_mm': anchor,
        'top_z_mm': float(hi[2]), 'bottom_z_mm': float(lo[2]),
        'plate_mm': {'min': [lo[0], lo[1]], 'max': [hi[0], hi[1]], 'top_z': plate_top},
    }, v, f


def load(step_path=DEFAULT_STEP, cache_dir=CACHE_DIR):
    """Station geometry (cached). Adds 'visual_stl': its mesh in metres, station frame."""
    key = _cache_key(step_path)
    out = os.path.join(cache_dir, key)
    meta_path = os.path.join(out, 'meta.json')
    if not os.path.exists(meta_path):
        import trimesh
        info, v, f = analyse(step_path)
        os.makedirs(out, exist_ok=True)
        for old in glob.glob(os.path.join(cache_dir, '*', 'meta.json')):
            if os.path.dirname(old) != out:
                os.remove(old)
        trimesh.Trimesh(v * 0.001, f, process=False).export(os.path.join(out, 'visual.stl'))
        with open(meta_path, 'w') as fh:
            json.dump(info, fh, indent=1)
    with open(meta_path) as fh:
        info = json.load(fh)
    info['visual_stl'] = os.path.join(out, 'visual.stl')
    return info


def _rot(yaw_deg):
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    return np.array([[c, -s], [s, c]])


def place(info, near_xy, yaw_deg=0.0):
    """Where the station goes: its screw holes on table holes, its anchor (column, else plate centre) as close to
    near_xy as that allows.

    Returns dict(pos_xy (m, station origin in the world), yaw_deg, pick_xy (m, column axis), support_z (m, top
    face), hole_error_mm (worst station hole vs its table hole), holes_world_mm).
    """
    R = _rot(yaw_deg)
    holes = np.asarray(info['holes_mm'], dtype=float) @ R.T          # station holes, rotated, mm
    column = R @ np.asarray(info.get('anchor_xy_mm') or info['column_xy_mm'], dtype=float)
    pitch = HOLE_GRID_PITCH * 1000.0
    g0, gmax = np.asarray(HOLE_GRID_ORIGIN) * 1000.0, np.asarray(HOLE_GRID_MAX) * 1000.0
    target = np.asarray(near_xy, dtype=float) * 1000.0

    def snap(p):
        return g0 + np.round((p - g0) / pitch) * pitch

    best = None
    # Put the first hole on each table hole around the target; keep the best fitting, nearest one.
    base = snap(target - column + holes[0])
    for i in range(-3, 4):
        for j in range(-3, 4):
            g = base + pitch * np.array([i, j])
            t = g - holes[0]                                           # station origin, mm
            world = holes + t
            on_grid = snap(world)
            if np.any(on_grid < g0 - 1e-6) or np.any(on_grid > gmax + 1e-6):
                continue                                               # a hole would miss the perforated area
            err = float(np.max(np.linalg.norm(world - on_grid, axis=1)))
            dist = float(np.linalg.norm(column + t - target))
            key = (round(err, 1), dist)
            if best is None or key < best[0]:
                best = (key, t, err, world)
    if best is None:
        raise ValueError(f'The pick station does not fit on the table hole grid near {near_xy}.')
    _, t, err, world = best
    # Bolted down, the plate settles between its screws: centre it on its table holes (least squares) instead of
    # putting the first hole exactly on one and letting the hole-spacing error pile up on the others.
    t = t + np.mean(snap(world) - world, axis=0)
    world = holes + t
    err = float(np.max(np.linalg.norm(world - snap(world), axis=1)))
    return {'pos_xy': (t / 1000.0).tolist(), 'yaw_deg': float(yaw_deg),
            'pick_xy': ((column + t) / 1000.0).tolist(),
            'support_z': (info['top_z_mm'] - info['bottom_z_mm']) / 1000.0,
            'hole_error_mm': round(err, 2), 'holes_world_mm': np.round(world, 1).tolist(),
            'table_holes': [[int(v) for v in np.round((h - g0) / pitch)] for h in snap(world)]}


def _abs(path):
    """Paths in config/cell.json are relative to the repo root."""
    return repo_path(path)


def is_seated_part(placement, step_path):
    """True when step_path goes on the fixture in its CAD frame: every part with pick_station.seat_all_parts
    (they all share the fixture's mounting), else only the part made for it (same contents as seated_part)."""
    if placement and placement.get('seat_all_parts'):
        return True
    if not placement or not placement.get('seated_part') or not os.path.exists(placement['seated_part']):
        return False
    from .step_to_mesh import file_hash
    return file_hash(step_path) == file_hash(placement['seated_part'])


def seated_pose(placement):
    """World (pos, quat wxyz) of a seated part: its STEP frame = the fixture's STEP frame, wherever the fixture is
    bolted down, turned about the fixture axis by placement['part_yaw_deg'] (how it was turned in the web view)."""
    yaw = np.radians(placement['yaw_deg'] + placement.get('part_yaw_deg', 0.0))
    tx, ty = placement['pos_xy']
    return [tx, ty, -placement['bottom_z_mm'] / 1000.0], [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]


def release_weld(model, data):
    """Lets the seated part go (after the gripper has closed on it). No-op without a fixture weld.

    The part's weight is compensated while the gripper holds it (like the payload setting of a real cobot):
    otherwise the position-servoed arm sags ~14 mm under it the moment the fixture lets go, and the part drops
    and swings sideways through the fixture instead of sliding straight up off it. drop_payload() ends that."""
    try:
        eq = model.equality(FIXTURE_WELD).id
    except KeyError:
        return False
    was = bool(data.eq_active[eq])
    data.eq_active[eq] = 0
    if was:
        from .cad_part import PART_BODY
        model.body_gravcomp[model.body(PART_BODY).id] = 1.0
    return was


def drop_payload(model):
    """The gripper lets go: the part has its weight again (no-op unless release_weld compensated it)."""
    from .cad_part import PART_BODY
    model.body_gravcomp[model.body(PART_BODY).id] = 0.0


def from_config(cfg):
    """(info, placement) for config/cell.json's pick_station, or (None, None) when it is off."""
    ps = cfg.get('pick_station') or {}
    if not ps.get('enabled', False):
        return None, None
    info = load(_abs(ps.get('file', DEFAULT_STEP)))
    placement = place(info, ps.get('near_xy', cfg['pick_xy']), ps.get('yaw_deg', 0.0))
    placement['seated_part'] = _abs(ps['seated_part']) if ps.get('seated_part') else None
    placement['bottom_z_mm'] = info['bottom_z_mm']
    placement['file'] = _abs(ps.get('file', DEFAULT_STEP))
    placement['seat_all_parts'] = bool(ps.get('seat_all_parts', False))
    if placement['hole_error_mm'] > MAX_HOLE_ERROR_MM:
        print(f"[pick_station] warning: its screw holes miss the table holes by up to "
              f"{placement['hole_error_mm']:.1f} mm", file=sys.stderr)
    return info, placement


def add_to_spec(spec, info, placement, seat_part=False):
    """Station as world geoms: the STEP mesh (visual) plus a box (plate), a cylinder (column) and, for a fixture,
    a cylinder (top disc) for physics. seat_part: the part is the one made for this fixture (seated_pose); the
    geoms then go on a static body so part <-> station contacts can be excluded, and a weld holds the part."""
    w = spec.worldbody.add_body(name=STATION_BODY) if seat_part else spec.worldbody
    yaw = np.radians(placement['yaw_deg'])
    quat = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
    tx, ty = placement['pos_xy']
    z0 = -info['bottom_z_mm'] / 1000.0                     # station bottom on the table top (z = 0)
    R = _rot(placement['yaw_deg'])

    def world_xy(p_mm):
        return (R @ (np.asarray(p_mm, dtype=float) / 1000.0) + [tx, ty]).tolist()

    spec.add_mesh(name='pickstation_visual', file=info['visual_stl'])
    # The fixture loaded as the part too (e.g. the scale): the part draws it, the fixture only collides.
    shown = 0.0 if placement.get('same_as_part') else 1.0
    w.add_geom(name='pickstation_visual', type=G.mjGEOM_MESH, meshname='pickstation_visual',
               pos=[tx, ty, z0], quat=quat, rgba=[0.55, 0.62, 0.75, shown], contype=0, conaffinity=0, group=0)
    hidden = dict(group=3, rgba=[0.55, 0.62, 0.75, 0.0])
    pl = info['plate_mm']
    lo, hi = np.asarray(pl['min']), np.asarray(pl['max'])
    plate_h = (pl['top_z'] - info['bottom_z_mm']) / 1000.0
    w.add_geom(name='pickstation_plate', type=G.mjGEOM_BOX,
               size=[*((hi - lo) / 2000.0).tolist(), plate_h / 2],
               pos=[*world_xy((lo + hi) / 2), plate_h / 2], quat=quat, **hidden)
    if info.get('column_xy_mm') is not None:
        col_h = (info['top_z_mm'] - pl['top_z']) / 1000.0
        w.add_geom(name='pickstation_column', type=G.mjGEOM_CYLINDER,
                   size=[info['column_radius_mm'] / 1000.0, col_h / 2],
                   pos=[*world_xy(info['column_xy_mm']), plate_h + col_h / 2], friction=[1.0, 0.005, 0.0001],
                   **hidden)
    elif not placement.get('same_as_part'):
        # No column: the fixture's own convex pieces (CoACD, as for parts), in its STEP frame. Not when the fixture is
        # loaded as the part too (the scale): the part's pieces are the same shape, and the fingers may touch those.
        from .step_to_mesh import convert
        meta = convert(placement['file'])
        for i, hull in enumerate(meta['collision_hulls']):
            m = spec.add_mesh(name=f'pickstation_hull_{i:02d}', file=os.path.join(meta['dir'], hull))
            w.add_geom(name=f'pickstation_hull_{i:02d}', type=G.mjGEOM_MESH, meshname=m.name, pos=[tx, ty, z0],
                       quat=quat, friction=[1.0, 0.005, 0.0001], **hidden)
    top = info.get('top_disc_mm')
    if top:
        h = (info['top_z_mm'] - top['z0']) / 1000.0
        w.add_geom(name='pickstation_top', type=G.mjGEOM_CYLINDER, size=[top['radius'] / 1000.0, h / 2],
                   pos=[*world_xy(info['column_xy_mm']), z0 + info['top_z_mm'] / 1000.0 - h / 2],
                   friction=[1.0, 0.005, 0.0001], **hidden)
    if seat_part:
        from .cad_part import PART_BODY
        spec.add_exclude(bodyname1=STATION_BODY, bodyname2=PART_BODY)
        # data = anchor (3), part pose in the station body frame (= world: the body sits at the origin; pos 3 +
        # quat 4), torquescale
        pos, quat = seated_pose(placement)
        eq = spec.add_equality(name=FIXTURE_WELD, type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                               name1=STATION_BODY, name2=PART_BODY, active=True)
        eq.data[:11] = [0.0, 0.0, 0.0, *pos, *quat, 1.0]


if __name__ == '__main__':
    cfg = load_config()
    cfg.setdefault('pick_station', {})['enabled'] = True
    info, p = from_config(cfg)
    print(json.dumps({k: v for k, v in info.items() if k != 'visual_stl'}, indent=1))
    print(json.dumps(p, indent=1))
