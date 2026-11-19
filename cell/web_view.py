"""
web_view.py - Streams the MuJoCo cell to the web app, so Sim and Real Sim are watched in the browser.

When a job starts (preview, execute, real, home), WebView writes into its run folder:

  scene.json     every visible geom of the scene (the same table, fixture, tray, arm, gripper and part as the
                 MuJoCo window): type, size, colour, pose, and for meshes a content hash. Bodies that never move
                 are drawn at their pose; the moving ones are listed in `bodies`.
  frames.bin     the live stream: frames of float32 [t, then per moving body x y z qw qx qy qz] (world frame, m),
                 appended at most `fps` times per second while the job runs.

Mesh vertices go to var/cache/webview/<hash>.bin once (uint32 nv, uint32 nf, float32 verts, uint32 faces) and are
reused by every run, so the browser downloads the table once. `T_view` maps world (m) into the part frame the web
viewer shows (m); the web scales it to mm.
"""
import hashlib
import json
import os

import mujoco
import numpy as np

from .paths import CACHE
from .table_base import FLOOR_Z

MESH_DIR = os.path.join(CACHE, 'webview')
VISIBLE_GROUPS = (0, 1, 2)           # what the MuJoCo viewer shows by default; group 3 = collision shapes
GEOM_TYPES = {mujoco.mjtGeom.mjGEOM_PLANE: 'plane', mujoco.mjtGeom.mjGEOM_SPHERE: 'sphere',
              mujoco.mjtGeom.mjGEOM_CAPSULE: 'capsule', mujoco.mjtGeom.mjGEOM_ELLIPSOID: 'ellipsoid',
              mujoco.mjtGeom.mjGEOM_CYLINDER: 'cylinder', mujoco.mjtGeom.mjGEOM_BOX: 'box',
              mujoco.mjtGeom.mjGEOM_MESH: 'mesh'}


def _moves(m, b):
    """A body moves if it, or a body above it, has a joint or is a mocap body."""
    while b > 0:
        if m.body_jntnum[b] > 0 or m.body_mocapid[b] >= 0:
            return True
        b = int(m.body_parentid[b])
    return False


def _mesh_blob(m, mesh_id):
    v0, nv = int(m.mesh_vertadr[mesh_id]), int(m.mesh_vertnum[mesh_id])
    f0, nf = int(m.mesh_faceadr[mesh_id]), int(m.mesh_facenum[mesh_id])
    verts = np.ascontiguousarray(m.mesh_vert[v0:v0 + nv], dtype='<f4')
    faces = np.ascontiguousarray(m.mesh_face[f0:f0 + nf], dtype='<u4')
    blob = np.array([nv, nf], dtype='<u4').tobytes() + verts.tobytes() + faces.tobytes()
    key = hashlib.sha256(blob).hexdigest()[:32]
    path = os.path.join(MESH_DIR, f'{key}.bin')
    if not os.path.exists(path):
        os.makedirs(MESH_DIR, exist_ok=True)
        tmp = f'{path}.{os.getpid()}.tmp'
        with open(tmp, 'wb') as f:
            f.write(blob)
        os.replace(tmp, path)
    return key


class WebView:
    def __init__(self, model, data, run_dir, T_view, label='', fps=30.0, look=None):
        m = self.m = model
        mujoco.mj_forward(m, data)
        self.bodies = [b for b in range(1, m.nbody) if _moves(m, b)]
        index = {b: i for i, b in enumerate(self.bodies)}
        meshes, geoms = {}, []
        for g in range(m.ngeom):
            kind = GEOM_TYPES.get(int(m.geom_type[g]))
            name = m.geom(g).name or ''
            rgba = m.mat_rgba[m.geom_matid[g]] if m.geom_matid[g] >= 0 else m.geom_rgba[g]
            if (kind is None or int(m.geom_group[g]) not in VISIBLE_GROUPS or rgba[3] <= 0
                    or name.startswith('ghost')):        # the target ghost: the web draws its own
                continue
            b = int(m.geom_bodyid[g])
            entry = {'type': kind, 'size': [float(v) for v in m.geom_size[g]], 'rgba': [float(v) for v in rgba],
                     # what it is, for the web's real-world colours: body, geom and material names
                     'bodyName': m.body(b).name, 'geomName': name,
                     'material': m.mat(m.geom_matid[g]).name if m.geom_matid[g] >= 0 else ''}
            if kind == 'mesh':
                mid = int(m.geom_dataid[g])
                if mid not in meshes:
                    meshes[mid] = _mesh_blob(m, mid)
                entry['mesh'] = meshes[mid]
            if b in index:      # pose relative to its moving body
                entry.update(body=index[b], pos=m.geom_pos[g].tolist(), quat=m.geom_quat[g].tolist())
            else:               # never moves: its world pose
                q = np.zeros(4)
                mujoco.mju_mat2Quat(q, data.geom_xmat[g])
                entry.update(body=None, pos=data.geom_xpos[g].tolist(), quat=q.tolist())
            geoms.append(entry)
        self.frame_floats = 1 + 7 * len(self.bodies)
        self.next_t, self.dt = None, 1.0 / fps
        self.frames_path = os.path.join(run_dir, 'frames.bin')
        self._f = open(self.frames_path, 'wb')         # a new job starts a new stream
        scene = {'version': 1, 'label': label, 'bodies': [m.body(b).name for b in self.bodies],
                 'frameFloats': self.frame_floats, 'T_view': np.asarray(T_view, dtype=float).tolist(),
                 'floorZ': FLOOR_Z, 'look': look or {},
                 'geoms': geoms}
        tmp = os.path.join(run_dir, 'scene.json.tmp')
        with open(tmp, 'w') as f:
            json.dump(scene, f)
        os.replace(tmp, os.path.join(run_dir, 'scene.json'))
        self.frame(data, 0.0, force=True)

    def frame(self, data, t, force=False):
        """Appends the moving bodies' poses at time t (s), at most fps times per second."""
        if self._f is None or (not force and self.next_t is not None and t < self.next_t):
            return
        self.next_t = t + self.dt
        out = np.empty(self.frame_floats, dtype='<f4')
        out[0] = t
        poses = np.hstack([data.xpos[self.bodies], data.xquat[self.bodies]])
        out[1:] = poses.ravel()
        self._f.write(out.tobytes())
        self._f.flush()

    def close(self):
        if self._f is not None:
            self._f.close()
            self._f = None
