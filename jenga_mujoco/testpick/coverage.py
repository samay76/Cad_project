"""
coverage.py - Which parts of the CAD surface the scan camera has seen, and which view to try next.

Reference: the part's visual mesh exactly as MuJoCo loaded it (m.mesh_vert / m.mesh_face of the part's visual
geom), moved into the part BODY frame with geom_pos / geom_quat (MuJoCo re-centres meshes when it loads them).

  Coverage(...)                    samples ~n points evenly on that surface, with face normals, all RED
  .scan(d)                         casts the camera's ray grid; points near a part hit, facing the camera,
                                   turn GREEN. Returns the hits (world) for drawing the beam.
  .predict(m, scratch)             how many RED points would be seen with the scene in `scratch`
                                   (arm at an IK solution, part at a candidate pose)
  .report(m, d)                    what is still red, and how much of it the gripper hides

Rays: mj_multiRay from the camera, the camera rig body excluded (bodyexclude). A ray counts only if its first
hit is the part's visual geom, so the gripper fingers block rays. mj_ray skips geoms with rgba alpha 0 (the
part's collision hulls and the table's collision boxes are such); the group mask is printed at start.

The camera pose is read from d.cam_xpos / d.cam_xmat every time (mj_camlight after mj_kinematics in the
scratch data), so a wrist-mounted camera only needs a different camera name.
"""
import numpy as np
import mujoco
from scipy.spatial import cKDTree

UNSCANNED, SCANNED = 0, 1


def _mat(quat):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(quat, dtype=float))
    return R.reshape(3, 3)


def part_mesh_in_body_frame(m, geom_id):
    """(vertices, faces) of a mesh geom, in its body frame (m)."""
    mid = int(m.geom_dataid[geom_id])
    v0, nv = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
    f0, nf = int(m.mesh_faceadr[mid]), int(m.mesh_facenum[mid])
    verts = np.asarray(m.mesh_vert[v0:v0 + nv], dtype=float)
    faces = np.asarray(m.mesh_face[f0:f0 + nf], dtype=np.int64)
    R = _mat(m.geom_quat[geom_id])
    return verts @ R.T + m.geom_pos[geom_id], faces


def axis_vector(name):
    return {'x': np.array([1.0, 0, 0]), 'y': np.array([0, 1.0, 0]), 'z': np.array([0, 0, 1.0])}[name]


class Coverage:
    def __init__(self, m, part_body, part_geom, cam_id, cam_body, scan_cfg, up_body, include_top_bottom,
                 scan_distance, seed=0):
        import trimesh
        self.part_body, self.part_geom, self.cam_id, self.cam_body = part_body, part_geom, cam_id, cam_body
        self.cfg = scan_cfg
        self.cos_inc = float(np.cos(np.radians(scan_cfg['max_incidence_deg'])))
        self.tol = float(scan_cfg.get('predict_tolerance_m', 0.001))

        verts, faces = part_mesh_in_body_frame(m, part_geom)
        mesh = trimesh.Trimesh(verts, faces, process=False)
        if mesh.is_volume and mesh.volume < 0:  # inward winding -> flip so normals point out
            mesh.invert()
        self.mesh = mesh
        pts, fidx = trimesh.sample.sample_surface_even(mesh, int(scan_cfg['n_points']), seed=seed)
        self.points = np.asarray(pts, dtype=float)
        self.normals = np.asarray(mesh.face_normals[fidx], dtype=float)
        self.up = np.asarray(up_body, dtype=float) / np.linalg.norm(up_body)
        if include_top_bottom:
            self.active = np.ones(len(self.points), bool)
        else:
            self.active = np.abs(self.normals @ self.up) <= float(scan_cfg['up_dot_max'])
        self.state = np.full(len(self.points), UNSCANNED, np.int8)
        self.tree = cKDTree(self.points)

        # Ray grid in the camera frame (MuJoCo cameras look along -Z, +Y up, +X right)
        W, H = int(scan_cfg['rays_w']), int(scan_cfg['rays_h'])
        ty = np.tan(np.radians(m.cam_fovy[cam_id]) / 2)
        tx = ty * W / H
        xs = ((np.arange(W) + 0.5) / W * 2 - 1) * tx
        ys = ((np.arange(H) + 0.5) / H * 2 - 1) * ty
        X, Y = np.meshgrid(xs, ys)
        dirs = np.stack([X.ravel(), Y.ravel(), -np.ones(X.size)], axis=1)
        self.ray_dirs_cam = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
        self.half_tan = (tx, ty)

        hr = scan_cfg.get('hit_radius_m', 'auto')
        spacing = 2 * ty / H * scan_distance
        self.hit_radius = float(scan_cfg.get('hit_radius_factor', 1.2) * spacing) if hr == 'auto' else float(hr)

        # Geom groups the rays see: every group that holds a geom (alpha-0 geoms are skipped by mj_ray anyway)
        groups = sorted(set(int(g) for g in m.geom_group if 0 <= g < 6))
        self.geomgroup = np.zeros(6, np.uint8)
        self.geomgroup[groups] = 1
        self._print_setup(m, groups, spacing)

    def _print_setup(self, m, groups, spacing):
        n, a = len(self.points), int(self.active.sum())
        print(f'[coverage] {n} surface points sampled; {n - a} on top/bottom faces excluded '
              f'(|n.UP| > {self.cfg["up_dot_max"]}, UP = {np.round(self.up, 3).tolist()} in the part frame); '
              f'{a} to scan', flush=True)
        print(f'[coverage] ray grid {self.cfg["rays_w"]}x{self.cfg["rays_h"]}, spacing {spacing * 1000:.2f} mm at '
              f'the scan distance -> HIT_RADIUS {self.hit_radius * 1000:.2f} mm, max incidence '
              f'{self.cfg["max_incidence_deg"]} deg', flush=True)
        for g in groups:
            ids = [i for i in range(m.ngeom) if m.geom_group[i] == g]
            visible = [i for i in ids if m.geom_rgba[i][3] > 0]
            names = sorted({(m.geom(i).name or m.body(m.geom_bodyid[i]).name).split('_0')[0] for i in visible})[:6]
            print(f'[coverage] ray geomgroup {g}: {len(ids)} geoms, {len(visible)} opaque (hit-able), e.g. {names}',
                  flush=True)
        print(f'[coverage] rays: first hit must be geom {m.geom(self.part_geom).name!r}; camera body '
              f'{m.body(self.cam_body).name!r} excluded; alpha-0 geoms (part hulls, table boxes) never hit', flush=True)

    # ---------------- bookkeeping ----------------
    def coverage(self):
        a = self.active
        return float((self.state[a] == SCANNED).sum()) / max(1, int(a.sum()))

    def n_red(self):
        return int(((self.state == UNSCANNED) & self.active).sum())

    def part_pose(self, d):
        return d.xpos[self.part_body].copy(), d.xmat[self.part_body].reshape(3, 3).copy()

    def camera_pose(self, d):
        return d.cam_xpos[self.cam_id].copy(), d.cam_xmat[self.cam_id].reshape(3, 3).copy()

    def _cast(self, m, d, origin, dirs):
        n = len(dirs)
        geomid = np.full(n, -1, np.int32)
        dist = np.full(n, -1.0)
        mujoco.mj_multiRay(m, d, origin, np.ascontiguousarray(dirs.ravel()), self.geomgroup, 1, self.cam_body,
                           geomid, dist, None, n, 10.0)
        return geomid, dist

    # ---------------- scan ----------------
    def scan(self, m, d):
        """Ray grid from the camera; marks seen points GREEN. Returns dict(hits_world, new, coverage, n_hits)."""
        cam_pos, cam_R = self.camera_pose(d)
        p, R = self.part_pose(d)                 # actual part pose, re-read every scan
        dirs = self.ray_dirs_cam @ cam_R.T
        geomid, dist = self._cast(m, d, cam_pos, dirs)
        on_part = (geomid == self.part_geom) & (dist > 0)
        hits_w = cam_pos + dirs[on_part] * dist[on_part, None]
        new = 0
        if len(hits_w):
            hits_b = (hits_w - p) @ R                         # world -> part body frame
            near = self.tree.query_ball_point(hits_b, self.hit_radius)
            idx = np.unique(np.fromiter((i for lst in near for i in lst), dtype=np.int64)) if len(near) else []
            idx = np.asarray(idx, dtype=np.int64)
            if idx.size:
                facing = self._facing(idx, p, R, cam_pos)
                cand = idx[facing & self.active[idx] & (self.state[idx] == UNSCANNED)]
                self.state[cand] = SCANNED
                new = int(cand.size)
        return dict(hits_world=hits_w, new=new, coverage=self.coverage(), n_hits=int(on_part.sum()),
                    cam_pos=cam_pos)

    def _facing(self, idx, p, R, cam_pos):
        pw = self.points[idx] @ R.T + p
        nw = self.normals[idx] @ R.T
        to_cam = cam_pos - pw
        to_cam /= np.linalg.norm(to_cam, axis=1, keepdims=True)
        return np.einsum('ij,ij->i', nw, to_cam) >= self.cos_inc

    # ---------------- prediction (next best view) ----------------
    def predict(self, m, scratch):
        """Number of RED points the camera would see in `scratch` (kinematics already set by the caller)."""
        mujoco.mj_kinematics(m, scratch)
        mujoco.mj_camlight(m, scratch)
        idx = np.nonzero((self.state == UNSCANNED) & self.active)[0]
        if idx.size == 0:
            return 0
        cam_pos, _ = self.camera_pose(scratch)
        p, R = self.part_pose(scratch)
        idx = idx[self._facing(idx, p, R, cam_pos)]
        if idx.size == 0:
            return 0
        pw = self.points[idx] @ R.T + p
        inside = self._in_view(scratch, pw)
        if not inside.any():
            return 0
        pw = pw[inside]
        v = pw - cam_pos
        expected = np.linalg.norm(v, axis=1)
        geomid, dist = self._cast(m, scratch, cam_pos, v / expected[:, None])
        visible = (geomid == self.part_geom) & (np.abs(dist - expected) < self.tol)
        return int(visible.sum())

    def _in_view(self, d, pw):
        cam_pos, cam_R = self.camera_pose(d)
        pc = (pw - cam_pos) @ cam_R              # world -> camera frame
        z = -pc[:, 2]
        tx, ty = self.half_tan
        return (z > 0) & (np.abs(pc[:, 0]) <= tx * z) & (np.abs(pc[:, 1]) <= ty * z)

    # ---------------- final report ----------------
    def report(self, m, d, gripper_bodies, probe_m=0.06):
        """Red points that remain, and how many of them are hidden by the gripper: a short ray from the point
        along its outward normal hits a gripper geom first."""
        idx = np.nonzero((self.state == UNSCANNED) & self.active)[0]
        p, R = self.part_pose(d)
        under_gripper = 0
        gid = np.array([-1], np.int32)
        for i in idx:
            pw = self.points[i] @ R.T + p
            nw = self.normals[i] @ R.T
            dist = mujoco.mj_ray(m, d, pw + nw * 2e-4, nw, self.geomgroup, 1, self.part_body, gid)
            if 0 <= dist <= probe_m and gid[0] >= 0 and m.geom_bodyid[gid[0]] in gripper_bodies:
                under_gripper += 1
        return dict(coverage=self.coverage(), active=int(self.active.sum()), red=int(idx.size),
                    red_under_gripper=under_gripper, excluded=int((~self.active).sum()))
