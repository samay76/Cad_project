"""
viz.py - Draws the coverage into the passive viewer's user scene (viewer.user_scn).

  - every coverage point as a small sphere at its CURRENT world position (it moves with the part):
    green = scanned, red = not yet; excluded top/bottom points as small dim grey dots
  - the camera's view cone: 4 faint edge lines + the far rectangle
  - during a scan: ~40 beam lines from the lens to part hit points

No mujoco.Renderer here: it crashes together with the passive viewer in one process. Look through the scan
camera with the viewer's camera dropdown (left panel -> Rendering -> Camera -> scan_cam).
"""
import numpy as np
import mujoco

from coverage import SCANNED

GREEN = np.array([0.1, 0.9, 0.25, 1.0], np.float32)
RED = np.array([0.95, 0.12, 0.1, 1.0], np.float32)
GREY = np.array([0.55, 0.55, 0.6, 0.35], np.float32)
BEAM = np.array([0.2, 0.85, 1.0, 0.8], np.float32)
CONE = np.array([1.0, 1.0, 0.4, 0.35], np.float32)
EYE3 = np.eye(3).ravel()


class Viz:
    def __init__(self, viewer, cov, viz_cfg, cone_length):
        self.viewer, self.cov = viewer, cov
        self.r = float(viz_cfg.get('point_radius_m', 0.0012))
        self.show_excluded = bool(viz_cfg.get('show_excluded', True))
        self.cone_length = cone_length
        self.beams = None  # (cam_pos, hit points) while a scan is shown

    def _line(self, scn, a, b, rgba, width):
        if scn.ngeom >= scn.maxgeom:
            return
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), EYE3, rgba)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, width, np.asarray(a, float), np.asarray(b, float))
        scn.ngeom += 1

    def draw(self, d):
        if self.viewer is None:
            return
        cov = self.cov
        p, R = cov.part_pose(d)
        pw = cov.points @ R.T + p
        cam_pos, cam_R = cov.camera_pose(d)
        with self.viewer.lock():
            scn = self.viewer.user_scn
            scn.ngeom = 0
            # view cone
            tx, ty = cov.half_tan
            L = self.cone_length
            corners = [cam_pos + cam_R @ (np.array([sx * tx, sy * ty, -1.0]) * L)
                       for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            for i, c in enumerate(corners):
                self._line(scn, cam_pos, c, CONE, 1.5)
                self._line(scn, c, corners[(i + 1) % 4], CONE, 1.0)
            # beams
            if self.beams is not None:
                origin, hits = self.beams
                for h in hits:
                    self._line(scn, origin, h, BEAM, 2.0)
            # coverage points
            size = np.array([self.r, 0, 0])
            small = np.array([self.r * 0.6, 0, 0])
            for i in range(len(pw)):
                if scn.ngeom >= scn.maxgeom:
                    break
                if cov.active[i]:
                    rgba, s = (GREEN if cov.state[i] == SCANNED else RED), size
                elif self.show_excluded:
                    rgba, s = GREY, small
                else:
                    continue
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, s, pw[i], EYE3, rgba)
                scn.ngeom += 1

    def show_beams(self, cam_pos, hits_world, n, rng):
        if len(hits_world) == 0:
            self.beams = None
            return
        pick = rng.choice(len(hits_world), size=min(n, len(hits_world)), replace=False)
        self.beams = (cam_pos, hits_world[pick])

    def clear_beams(self):
        self.beams = None

    def text(self, title, detail):
        if self.viewer is not None:
            self.viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                    title, detail)])
