"""
grasp_planner.py - Parallel-jaw grasp candidates computed from the part's geometry.

Pure geometry (no robot, no simulation): everything here is in the part's CAD frame, in meters.
Candidates that survive are checked against the real robot and scene by auto_pick_place.py.

How a grasp is found, given the point the user clicked:
  1. Sample surface points, most of them near the click.
  2. Cast a ray from each point along its surface normal to find the opposite surface:
       into the material -> outside grip (fingers close on two outer faces),
       into free space   -> inside grip  (closed fingers enter a bore or slot, then open against its walls).
     Keep pairs whose normals oppose each other (inside the friction cone) and that fit the gripper.
     Clicking the wall of a bore asks for an inside grip (see gap_at_click). A grip across a wall, with
     one finger in the cavity and the other outside, is never used.
  3. Spin the approach direction around the closing axis. Keep directions the robot can use (not from
     below), put the gripping face on the contact points, and back off if the fingertips would reach
     the table.
  4. Measure how the real pad footprint lands on the part: the true opening needed (a feature wider
     than the contact pair widens it), recentred so closing does not shove the part, and the fraction
     of the pad that touches.
  5. Score (click distance, normals, centre of mass, pad contact, approach, margin) and return a
     varied, ranked list.
"""
from collections import Counter
from dataclasses import dataclass, field

import numpy as np

EPS_T = 1e-5


@dataclass
class GraspParams:
    n_samples: int = 1500
    near_fraction: float = 0.6
    friction: float = 0.5           # friction cone for the antipodal test
    max_tilt_deg: float = 65.0      # approach direction from straight down
    max_tilt_inside_deg: float = 90.0  # a bore is entered along its axis, even if the part lies on its side
    n_approach: int = 16
    table_clearance_m: float = 0.003
    max_backoff_m: float = 0.015    # how far the pad may slide off the contact to clear the table
    min_width_m: float = 0.002
    open_margin_m: float = 0.004    # the fingers must open this much wider than the part
    finger_fit_m: float = 0.002     # clearance on each side for fingers entering a bore
    max_stop_short_m: float = 0.006 # how far from a marked point a finger may stop on a curved surface
    n_refine: int = 250             # candidates measured with the full pad footprint
    modes: tuple = ('outside', 'inside')
    n_return: int = 40


@dataclass
class Candidate:
    T_part_tcp: np.ndarray
    width_mm: float                 # opening at contact (outside) or the bore width (inside)
    mode: str                       # 'outside' or 'inside'
    contacts: tuple                 # two contact points, part frame (m)
    score: float = 0.0
    terms: dict = field(default_factory=dict)


class RayMesh:
    """Numpy ray casting (Moller-Trumbore) with a cheap distance cull, no extra dependencies."""

    def __init__(self, vertices, faces):
        tri = np.asarray(vertices, dtype=float)[np.asarray(faces)]
        self._set(tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0], tri.mean(axis=1))
        n = np.cross(self.e1, self.e2)
        norm = np.linalg.norm(n, axis=1)
        self.areas = norm / 2
        self.normals = n / np.maximum(norm, 1e-15)[:, None]
        self.radius = np.linalg.norm(tri - self.centroid[:, None, :], axis=2).max(axis=1)

    def _set(self, v0, e1, e2, centroid):
        self.v0, self.e1, self.e2, self.centroid = v0, e1, e2, centroid

    def near_line(self, origin, direction, radius):
        """Sub-mesh of the triangles within `radius` of a line (speeds up many parallel rays)."""
        rel = self.centroid - origin
        off = rel - np.outer(rel @ direction, direction)
        mask = np.einsum('ij,ij->i', off, off) <= (self.radius + radius) ** 2
        sub = RayMesh.__new__(RayMesh)
        sub._set(self.v0[mask], self.e1[mask], self.e2[mask], self.centroid[mask])
        sub.normals, sub.radius = self.normals[mask], self.radius[mask]
        return sub

    def hits(self, origin, direction):
        """(distance, triangle index) of every hit along the ray, nearest first."""
        rel = self.centroid - origin
        off = rel - np.outer(rel @ direction, direction)
        keep = np.flatnonzero(np.einsum('ij,ij->i', off, off) <= self.radius ** 2 * 1.0001 + 1e-12)
        if keep.size == 0:
            return np.empty(0), keep
        e1, e2 = self.e1[keep], self.e2[keep]
        p = np.cross(direction, e2)
        det = np.einsum('ij,ij->i', e1, p)
        ok = np.abs(det) > 1e-15
        keep, e1, e2, p, det = keep[ok], e1[ok], e2[ok], p[ok], det[ok]
        s = origin - self.v0[keep]
        u = np.einsum('ij,ij->i', s, p) / det
        q = np.cross(s, e1)
        v = (q @ direction) / det
        t = np.einsum('ij,ij->i', e2, q) / det
        ok = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > EPS_T)
        t, keep = t[ok], keep[ok]
        order = np.argsort(t)
        return t[order], keep[order]

    def first_facing(self, origin, direction, leaving):
        """First hit whose surface faces away from (leaving=True) or towards the ray: (t, normal) or None."""
        t, idx = self.hits(origin, direction)
        for ti, i in zip(t, idx):
            facing = self.normals[i] @ direction
            if (facing > 0) == leaving and abs(facing) > 1e-6:
                return ti, self.normals[i]
        return None


def sample_surface(mesh, click, params, rng):
    """Surface points and outward normals: the click first, most of the rest close to it."""
    probs = mesh.areas / mesh.areas.sum()
    n_near = int(params.n_samples * params.near_fraction)
    n_far = params.n_samples - n_near
    idx = [rng.choice(len(probs), n_far, p=probs)]
    if click is not None and n_near > 0:
        diag = np.ptp(mesh.centroid, axis=0)
        radius = max(0.012, 0.25 * float(np.linalg.norm(diag)))
        near = np.flatnonzero(np.linalg.norm(mesh.centroid - click, axis=1) < radius + mesh.radius)
        if near.size:
            idx.append(rng.choice(near, n_near, p=probs[near] / probs[near].sum()))
    idx = np.concatenate(idx)
    a, b = rng.random((2, idx.size))
    flip = a + b > 1
    a[flip], b[flip] = 1 - a[flip], 1 - b[flip]
    pts = mesh.v0[idx] + a[:, None] * mesh.e1[idx] + b[:, None] * mesh.e2[idx]
    return pts, mesh.normals[idx]


def gap_at_click(vertices, faces, click_point, click_normal, params=None):
    """True if the clicked surface faces the opposite wall across free space, as the inside of a bore does."""
    if click_point is None or click_normal is None:
        return False
    params = params or GraspParams()
    mesh = RayMesh(vertices, faces)
    t, _ = mesh.hits(click_point + click_normal * 1e-3, -click_normal)
    if not (t.size and abs(t[0] - 1e-3) < 5e-4):  # not a surface point (parts-list selection)
        return False
    hit = mesh.first_facing(click_point + click_normal * 1e-4, click_normal, leaving=False)
    return hit is not None and float(-(click_normal @ hit[1])) >= np.cos(np.arctan(params.friction))


def _in_cavity(mesh, point, outward):
    """True if leaving the surface at `point` runs into the part again: a finger here sits in a cavity."""
    return mesh.first_facing(point + outward * 1e-4, outward, leaving=False) is not None


def _opposite_surfaces(mesh, point, direction, mode):
    """(distance, normal) of the surfaces a finger opposite `point` could press on.
    Outside grip: where the ray first leaves the material, and where it finally leaves the part. On a tube
    the first is the bore, the last is the far side: only the last spans the part with both fingers outside."""
    if mode == 'inside':
        hit = mesh.first_facing(point, direction, leaving=False)
        return [hit] if hit is not None else []
    t, idx = mesh.hits(point, direction)
    leaving = [(ti, mesh.normals[i]) for ti, i in zip(t, idx) if mesh.normals[i] @ direction > 1e-6]
    if not leaving:
        return []
    return leaving[:1] if len(leaving) == 1 else [leaving[0], leaving[-1]]


def antipodal_pairs(mesh, points, normals, geo, params, rejects):
    """(p_a, p_b, closing axis, width, mode, cos angle) for surface pairs the gripper could hold."""
    cone = np.cos(np.arctan(params.friction))
    max_w = geo.max_opening_mm / 1000.0
    pairs, seen = [], set()
    for p, n in zip(points, normals):
        for mode, d in (('outside', -n), ('inside', n)):
            if mode not in params.modes:
                continue
            for t, n2 in _opposite_surfaces(mesh, p, d, mode):
                if t < params.min_width_m:
                    continue
                cos_a = float(-(n @ n2))
                if mode == 'outside' and t > max_w - params.open_margin_m:
                    rejects['wider than the gripper opening'] += 1
                    continue
                # The closed fingers' outer faces are what must fit inside a bore.
                if mode == 'inside':
                    if t > max_w + 2 * geo.at(geo.max_opening_mm).outer_x:
                        continue
                    if t < 2 * (geo.at(0.0).outer_x + params.finger_fit_m):
                        rejects['gap too narrow for the fingers'] += 1
                        continue
                if cos_a < cone:
                    rejects['surfaces not opposite enough to hold'] += 1
                    continue
                q = p + d * t
                if mode == 'outside' and _in_cavity(mesh, p, n) != _in_cavity(mesh, q, n2):
                    rejects['grip across a wall, one finger in a cavity'] += 1
                    continue
                key = (mode, *np.round((p + q) / 2 / 0.0015).astype(int), *np.round(d / 0.1).astype(int))
                if key in seen:
                    continue
                seen.add(key)
                pairs.append((p, q, d, float(t), mode, cos_a))
    return pairs


def _pose(c_tcp, x, z):
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, c_tcp
    return T


def _orthonormal(x):
    u = np.cross(x, [0.0, 0.0, 1.0] if abs(x[2]) < 0.9 else [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    return u, np.cross(x, u)


def _lowest_tip_z(T_world_tcp, pad):
    """Lowest fingertip corner in world Z (pad = the finger shape at the current opening)."""
    x, y, z, o = T_world_tcp[:3, 0], T_world_tcp[:3, 1], T_world_tcp[:3, 2], T_world_tcp[:3, 3]
    reach = abs(x[2]) * pad.outer_x + abs(y[2]) * pad.half_y
    return float((o + z * pad.tip_z)[2] - reach)


def footprint(mesh, T, pad, half_width, n=5, reach=0.03):
    """Lays the pad rectangle on the part from both sides. Returns (offset of the part centre along X,
    width at first touch, fraction of the pad that touches), or None if a side misses the part."""
    x, y, z, o = T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3]
    center = o + z * pad.center_z
    sub = mesh.near_line(center, x, np.hypot(pad.half_y, pad.half_z) + 1e-4)
    reach = half_width + reach
    stops, touched = [], 0.0
    for sign in (1.0, -1.0):
        dists = []
        for a in np.linspace(-pad.half_y, pad.half_y, n):
            for b in np.linspace(-pad.half_z, pad.half_z, n):
                origin = center + y * a + z * b + x * sign * reach
                t, _ = sub.hits(origin, -sign * x)
                if t.size:
                    dists.append(reach - t[0])   # position of the part surface from the centre plane
        if not dists:
            return None
        d = np.array(dists)
        stops.append(float(d.max()))
        touched += float(np.mean(d > d.max() - 0.0015)) * (d.size / n ** 2)
    offset = (stops[0] - stops[1]) / 2
    return offset, stops[0] + stops[1], touched / 2


def generate_candidates(vertices, faces, click_point, click_normal, T_world_part, com, geo, params=None,
                        table_z=0.0, seed=0):
    """Returns (ranked candidates, Counter of why samples were rejected)."""
    params = params or GraspParams()
    rng = np.random.default_rng(seed)
    mesh = RayMesh(vertices, faces)
    rejects = Counter()
    R_wp = T_world_part[:3, :3]
    max_cos_tilt = {'outside': np.cos(np.radians(params.max_tilt_deg)),
                    'inside': np.cos(np.radians(params.max_tilt_inside_deg))}
    diag = float(np.linalg.norm(np.ptp(np.asarray(vertices), axis=0)))
    click_scale = max(0.01, 0.2 * diag)
    com = np.asarray(com, dtype=float)

    points, normals = sample_surface(mesh, click_point, params, rng)
    if click_point is not None and click_normal is not None:
        # Selecting a part in the parts list gives its centre, not a surface point: nothing to seed from.
        t, _ = mesh.hits(click_point + click_normal * 1e-3, -click_normal)
        if t.size and abs(t[0] - 1e-3) < 5e-4:
            points = np.vstack([click_point, points])
            normals = np.vstack([click_normal, normals])
    pairs = antipodal_pairs(mesh, points, normals, geo, params, rejects)

    prelim = []
    cone = np.cos(np.arctan(params.friction))
    for p, q, d, width, mode, cos_a in pairs:
        x = d
        center = (p + q) / 2
        u, v = _orthonormal(x)
        width_mm = width * 1000.0
        pad = geo.at(width_mm if mode == 'outside' else 0.0)
        s_anti = (cos_a - cone) / (1 - cone)
        # Distance from the centre of mass to the grip point: a far grip lets the weight twist the part.
        s_com = float(np.exp(-np.linalg.norm(com - center) / max(0.01, 0.25 * diag)))
        angles = 2 * np.pi * np.arange(params.n_approach) / params.n_approach
        directions = list(np.outer(np.cos(angles), u) + np.outer(np.sin(angles), v))
        # A fixed set of angles would miss the best one: add straight down as far as the closing axis allows.
        down = R_wp.T @ np.array([0.0, 0.0, -1.0])
        down = down - (down @ x) * x
        if np.linalg.norm(down) > 1e-6:
            directions.insert(0, down / np.linalg.norm(down))
        for z in directions:
            z_w = R_wp @ z
            vertical = -z_w[2]
            if vertical < max_cos_tilt[mode] - 1e-9:
                rejects['approach would come from the side or below'] += 1
                continue
            # Fingers entering a bore need a clear line in: behind the grip point there must be no wall.
            if mode == 'inside' and mesh.hits(center, -z)[0].size:
                rejects['a wall blocks the way into the opening'] += 1
                continue
            T = _pose(center - z * pad.center_z, x, z)
            T_w = T_world_part @ T
            low = _lowest_tip_z(T_w, pad)
            needed = table_z + params.table_clearance_m - low
            # Backing off along a level approach does not raise the fingers at all.
            if needed > 0 and (vertical < 0.05 or needed / vertical > min(params.max_backoff_m, pad.half_z + 0.002)):
                rejects['fingertips would reach the table'] += 1
                continue
            backoff = needed / vertical if needed > 0 else 0.0
            T[:3, 3] -= z * backoff

            s_click = 1.0
            s_normal = 0.0
            if click_point is not None:
                rel = click_point - center
                in_plane = rel - (rel @ z) * z
                along_x = abs(in_plane @ x)
                other = in_plane - (in_plane @ x) * x
                dist = np.hypot(max(0.0, along_x - width / 2), np.linalg.norm(other))
                s_click = float(np.exp(-dist / click_scale))
                if click_normal is not None:
                    s_normal = max(float(-(click_normal @ z)), abs(float(click_normal @ x)))
            score = (3.0 * s_click + 1.5 * s_anti + 1.5 * s_com + 1.5 * vertical ** 2 + 1.0 * s_normal
                     + 0.5 * (1 - width_mm / max(geo.max_opening_mm, width_mm))
                     - (1.5 if mode == 'inside' else 0.0))
            prelim.append((score, T, width, mode, (p, q), dict(
                click=s_click, antipodal=s_anti, com=s_com, vertical=float(vertical), normal=s_normal)))

    prelim.sort(key=lambda c: -c[0])
    measured = []
    for score, T, width, mode, contacts, terms in prelim[:params.n_refine]:
        width_mm = width * 1000.0
        if mode == 'outside':
            pad = geo.at(width_mm)
            fp = footprint(mesh, T, pad, width / 2)
            if fp is None:
                rejects['finger pad misses the part'] += 1
                continue
            offset, real_width, cover = fp
            if real_width > geo.max_opening_mm / 1000.0 - params.open_margin_m:
                rejects['wider than the gripper opening'] += 1
                continue
            T = T.copy()
            T[:3, 3] += T[:3, 0] * offset
            width_mm = real_width * 1000.0
        else:
            cover = 0.3
        terms = dict(terms, contact=cover)
        measured.append(Candidate(T, width_mm, mode, contacts, score + 2.0 * cover, terms))
    measured.sort(key=lambda c: -c.score)
    return _diverse(measured, params.n_return), rejects


class PointsError(Exception):
    """The two marked points cannot be gripped, whatever the robot does. The message says why."""


def classify_points(p1, n1, p2, n2, friction):
    """The grip two surface points call for: (mode, width, closing axis, offset of p1 from p2 across the fingers).

    The fingers are flat and close along one axis, so what matters is how the two surfaces face each other, not
    the line between the points: the points need not be lined up, as long as both land on the pads. The closing
    axis splits the difference between the two surface normals, which is the best any grip can do.
    Surfaces facing away from each other: the fingers close on them (outside).
    Facing each other, as in a bore: the fingers open against them (inside)."""
    if np.linalg.norm(p2 - p1) < 1e-3:
        raise PointsError('The two points are in the same place. Mark one point for each finger.')
    opposition = float(-(n1 @ n2))
    if opposition <= 0.0:
        raise PointsError('The two surfaces face the same way, so the fingers cannot squeeze between them. Mark points '
                          'on surfaces that face opposite directions, such as two sides of a wall or of a bore.')
    apart = float(np.degrees(np.arccos(np.clip(opposition, -1, 1))))
    limit = 2 * float(np.degrees(np.arctan(friction)))
    if apart > limit:
        raise PointsError(f'The two surfaces are {apart:.0f} deg from facing directly opposite (the limit is {limit:.0f} '
                          'deg), so the fingers would slide off. Mark points on surfaces that face each other more directly.')
    x = (n1 - n2) / np.linalg.norm(n1 - n2)
    across = float((p1 - p2) @ x)
    if abs(across) < 1e-3:
        raise PointsError('The two points are side by side, with no thickness between them to grip.')
    return ('outside' if across > 0 else 'inside'), abs(across), x, (p1 - p2) - across * x


def grasps_from_points(vertices, faces, points, T_world_part, geo, params=None, table_z=0.0):
    """Grasps that put one finger on each marked point: [(p, n), (p, n)] in the part frame (m).
    Returns (candidates, Counter of why poses were rejected, note for the user). Raises PointsError."""
    params = params or GraspParams()
    (p1, n1), (p2, n2) = points
    mode, width, axis, offset = classify_points(p1, n1, p2, n2, params.friction)
    mesh = RayMesh(vertices, faces)
    width_mm = width * 1000.0
    opening = geo.max_opening_mm
    note = ''

    if mode == 'outside':
        if width_mm > opening - params.open_margin_m * 1000:
            raise PointsError(f'The surfaces at the points are {width_mm:.1f} mm apart, but the gripper opens only {opening:.0f} mm.')
        if _in_cavity(mesh, p1, n1) != _in_cavity(mesh, p2, n2):
            note = 'One point is inside a cavity, so one finger goes in and the other stays outside. '
    else:
        closed, widest = 2000 * geo.at(0.0).outer_x, 2000 * geo.at(opening).outer_x
        if width_mm < closed + params.finger_fit_m * 2000:
            raise PointsError(f'The gap between the points is {width_mm:.1f} mm, but the closed fingers need '
                              f'{closed + params.finger_fit_m * 2000:.1f} mm to enter.')
        if width_mm > widest:
            raise PointsError(f'The gap between the points is {width_mm:.1f} mm, but the open fingers reach only {widest:.0f} mm.')
        line = (p2 - p1) / np.linalg.norm(p2 - p1)
        hit = mesh.first_facing(p1 + line * 1e-4, line, leaving=False)
        if hit is not None and hit[0] < np.linalg.norm(p2 - p1) - 0.002:
            raise PointsError(f'There is material between the two points, {hit[0] * 1000:.0f} mm from point 1.')

    center = (p1 + p2) / 2
    pad = pad_entry = geo.at(width_mm if mode == 'outside' else 0.0)  # inside: the fingers enter closed
    R_wp = T_world_part[:3, :3]
    max_tilt = params.max_tilt_inside_deg if mode == 'inside' else params.max_tilt_deg
    cos_tilt = np.cos(np.radians(max_tilt))
    angles = 2 * np.pi * np.arange(params.n_approach) / params.n_approach

    rejects, out = Counter(), []
    limit_y = geo.at(width_mm if mode == 'outside' else 0.0).half_y
    limit_z = pad.half_z
    # Whichever way the gripper turns, the offset between the points must fit across the pad rectangle.
    if np.linalg.norm(offset) / 2 > np.hypot(limit_y, limit_z):
        raise PointsError(f'The points are {np.linalg.norm(offset) * 1000:.0f} mm apart sideways, too far to both land on the '
                          f'fingers (each finger pad is about {2000 * limit_y:.0f} x {2000 * limit_z:.0f} mm). Mark them closer together.')
    for x in (axis, -axis):  # either finger can take either point
        u, v = _orthonormal(x)
        directions = list(np.outer(np.cos(angles), u) + np.outer(np.sin(angles), v))
        down = R_wp.T @ np.array([0.0, 0.0, -1.0])
        down = down - (down @ x) * x
        if np.linalg.norm(down) > 1e-6:
            directions.insert(0, down / np.linalg.norm(down))
        for z in directions:
            vertical = float(-(R_wp @ z)[2])
            if vertical < cos_tilt - 1e-9:
                rejects['approach would come from the side or below'] += 1
                continue
            if mode == 'inside' and mesh.hits(center, -z)[0].size:
                rejects['a wall blocks the way to the points'] += 1
                continue
            # The points may land anywhere on the gripping face, so the gripper can sit higher or lower.
            T0 = _pose(center - z * pad.center_z, x, z)
            # Both points must land on the pads: each sits half the offset from the middle of the finger.
            off_y, off_z = abs(offset @ T0[:3, 1]) / 2, abs(offset @ z) / 2
            if off_y > pad.half_y:
                rejects['the points are too far apart sideways to both land on the fingers'] += 1
                continue
            low = _lowest_tip_z(T_world_part @ T0, pad_entry)
            base = 0.0
            if low < table_z + params.table_clearance_m and vertical > 0.05:
                base = (table_z + params.table_clearance_m - low) / vertical
            for k in (0.0, 0.4, 0.8, -0.4, -0.8):
                shift = base + k * pad.half_z
                if abs(shift) + off_z > 0.95 * pad.half_z:
                    continue
                T = T0.copy()
                T[:3, 3] -= z * shift
                if _lowest_tip_z(T_world_part @ T, pad_entry) < table_z + params.table_clearance_m:
                    rejects['fingertips would reach the table'] += 1
                    continue
                if mode == 'outside':
                    # A flat pad meets a curved wall at its edges first, so the fingers stop a little short of
                    # the point. A long way short means something else is in the way.
                    fp = footprint(mesh, T, pad, width / 2, reach=0.012)
                    if fp is not None:
                        center_shift, real_width, _ = fp
                        if real_width > width + 2 * params.max_stop_short_m:
                            rejects['another feature stops the fingers before they reach the points'] += 1
                            continue
                        if real_width * 1000 > opening - params.open_margin_m * 1000:
                            rejects['wider than the gripper opening'] += 1
                            continue
                        T[:3, 3] += x * center_shift
                        hold = real_width * 1000
                    else:
                        hold = width_mm
                else:
                    hold = width_mm
                score = 3.0 * vertical ** 2 - abs(shift) / pad.half_z
                out.append(Candidate(T, hold, mode, (p1, p2), score, dict(vertical=vertical)))
    out.sort(key=lambda c: -c.score)
    return _diverse(out, params.n_return), rejects, note


def _diverse(ranked, count, min_mm=4.0, min_deg=20.0):
    """Best-first selection that skips near-duplicates, so later checks do not retry the same failure."""
    out = []
    for c in ranked:
        for o in out:
            near = np.linalg.norm(c.T_part_tcp[:3, 3] - o.T_part_tcp[:3, 3]) * 1000 < min_mm
            cos = np.clip((np.trace(c.T_part_tcp[:3, :3].T @ o.T_part_tcp[:3, :3]) - 1) / 2, -1, 1)
            if near and np.degrees(np.arccos(cos)) < min_deg:
                break
        else:
            out.append(c)
            if len(out) == count:
                break
    return out
