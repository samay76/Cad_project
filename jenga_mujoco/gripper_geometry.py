"""
gripper_geometry.py - Finger geometry of the loaded gripper, measured from the compiled MuJoCo model.

The grasp planner must know where the gripping faces really are, and that depends on the opening: the
2F-85's 4-bar linkage pulls the pads ~13 mm back towards the flange when fully open. So nothing is
hard-coded here. The collision geoms of the left finger are measured in the TCP frame over the range
of openings, and interpolated.

TCP frame (frames.py): +Z = approach, X = closing axis, origin between the fingertips.
"""
from dataclasses import dataclass

import numpy as np
import mujoco

from frames import inv, site_T
from ik import N_ARM

FACE_TOL = 0.0015  # a geom belongs to the gripping face if it comes this close to the inner limit (m)
_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


@dataclass(frozen=True)
class PadShape:
    """Gripping face of one finger at one opening, in the TCP frame (meters)."""
    center_z: float   # middle of the face along the approach axis (negative = behind the TCP)
    half_y: float     # half width across the closing plane
    half_z: float     # half length along the approach axis
    tip_z: float      # furthest point of the finger along the approach axis
    outer_x: float    # outermost point of the fingertip, measured from the TCP along X


class GripperGeometry:
    def __init__(self, model, fingers, home, tcp_site='g_tcp'):
        self.max_opening_mm = float(fingers.max_width)
        self.widths = np.linspace(0.0, self.max_opening_mm, 13)
        geoms = [g for g in range(model.ngeom)
                 if model.geom_contype[g] and model.body(model.geom_bodyid[g]).name.startswith('g_left')]
        pads = [g for g in geoms if model.geom(g).name.startswith('g_left_pad')]
        if not pads:
            raise RuntimeError('No finger pad geoms found in the gripper model.')

        data = mujoco.MjData(model)
        rows = []
        for width in self.widths:
            data.qpos[:N_ARM] = home
            fingers.set_qpos(data.qpos, width)
            mujoco.mj_kinematics(model, data)
            rows.append(self._measure(model, data, pads, geoms, tcp_site))
        self._table = np.array(rows)

    @staticmethod
    def _bounds(model, data, geom, T_tcp_world):
        """Axis-aligned bounds of a geom in the TCP frame, from its local bounding box."""
        center, half = model.geom_aabb[geom][:3], model.geom_aabb[geom][3:]
        R, p = data.geom_xmat[geom].reshape(3, 3), data.geom_xpos[geom]
        world = (center + _CORNERS * half) @ R.T + p
        tcp = world @ T_tcp_world[:3, :3].T + T_tcp_world[:3, 3]
        return tcp.min(axis=0), tcp.max(axis=0)

    def _measure(self, model, data, pads, geoms, tcp_site):
        T_tcp_world = inv(site_T(model, data, tcp_site))
        pad_bounds = [self._bounds(model, data, g, T_tcp_world) for g in pads]
        # The 'left' finger may lie on either side of the TCP depending on the model's frames.
        side = np.sign(np.mean([lo[0] + hi[0] for lo, hi in pad_bounds])) or 1.0
        inner = min(min(abs(lo[0]), abs(hi[0])) for lo, hi in pad_bounds)
        face = [b for b in pad_bounds if min(abs(b[0][0]), abs(b[1][0])) < inner + FACE_TOL]
        lo = np.min([b[0] for b in face], axis=0)
        hi = np.max([b[1] for b in face], axis=0)
        all_bounds = [self._bounds(model, data, g, T_tcp_world) for g in geoms]
        tip_z = max(b[1][2] for b in pad_bounds)
        near_tip = [b for b in all_bounds if b[1][2] > tip_z - 0.010]
        outer_x = max(max(side * b[0][0], side * b[1][0]) for b in near_tip)
        return [(lo[2] + hi[2]) / 2, (hi[1] - lo[1]) / 2, (hi[2] - lo[2]) / 2, tip_z, outer_x]

    def at(self, width_mm):
        return PadShape(*(float(np.interp(width_mm, self.widths, self._table[:, i])) for i in range(5)))
