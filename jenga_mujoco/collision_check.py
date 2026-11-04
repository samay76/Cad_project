"""
collision_check.py - Kinematic collision queries for planning (no physics step).

Uses the same MuJoCo collision geoms as the simulation, so a plan that passes here sees the shapes the
physics sees. Works on a private copy of the model with a contact margin, which makes MuJoCo report
near misses as well, so free-space motion can demand real clearance rather than just "not touching".

Checked for a joint configuration:
  arm/gripper vs table, tray, posts   (the arm's base bodies may touch the table)
  arm/gripper vs the part             (when carried, only the arm: the gripper is meant to touch it)
  arm vs itself and vs the gripper    (the gripper's own moving parts are not checked against each other)
  carried part vs table, tray, posts
"""
import copy
from dataclasses import dataclass

import numpy as np
import mujoco

from frames import T_to_pos_quat
from ik import N_ARM

MARGIN_M = 0.02        # contacts are reported up to this distance; must exceed every clearance used
SELF_TOL_M = 0.001


@dataclass(frozen=True)
class Hit:
    kind: str          # 'environment' | 'part' | 'self' | 'carried part'
    a: str
    b: str
    dist_mm: float     # negative = penetration
    pad: bool = False  # a finger pad touching the part

    def __str__(self):
        how = f'penetrates by {-self.dist_mm:.1f} mm' if self.dist_mm < 0 else f'only {self.dist_mm:.1f} mm away'
        return f"'{self.a}' vs '{self.b}': {how}"


class SceneCollision:
    def __init__(self, model, fingers, robot, part_body, part_joint):
        self.model = copy.copy(model)
        self.model.geom_margin[:] = MARGIN_M
        self.data = mujoco.MjData(self.model)
        self.fingers = fingers
        m = self.model
        self.part_bid = m.body(part_body).id
        self.part_adr = int(m.jnt_qposadr[m.joint(part_joint).id])
        root = m.body(robot.base_bodies[0]).id
        self.robot_body = np.array([m.body_rootid[b] == root for b in range(m.nbody)])
        self.gripper_body = np.array([m.body(b).name.startswith('g_') for b in range(m.nbody)])
        self.base_bodies = {m.body(b).id for b in robot.base_bodies}
        self.pad_geoms = {g for g in range(m.ngeom) if m.geom(g).name.startswith(('g_left_pad', 'g_right_pad'))}
        self.n_checks = 0

    def set_part(self, T_world_part):
        p, q = T_to_pos_quat(T_world_part)
        self.data.qpos[self.part_adr:self.part_adr + 3] = p
        self.data.qpos[self.part_adr + 3:self.part_adr + 7] = q

    def _name(self, geom):
        m = self.model
        body = m.geom_bodyid[geom]
        return m.body(body).name if body != 0 else (m.geom(geom).name or 'world')

    def check(self, q, width_mm, env_clearance=0.0, part_tol=0.001, ignore_part=False, ignore_pad_contact=False,
              carried_T_tcp_part=None, carried_clearance=0.0, tcp_site='g_tcp'):
        """First violated constraint for arm posture q and the fingers at width_mm, or None.

        env_clearance      : robot must stay this far from the environment (m)
        part_tol           : allowed penetration into the part when it is not carried (m)
        ignore_part        : skip robot vs part (e.g. while the fingers are closing on it)
        ignore_pad_contact : skip only the finger pads vs part
        carried_T_tcp_part : the part is held at this pose relative to the TCP
        carried_clearance  : carried part must stay this far from the environment (m)
        """
        m, d = self.model, self.data
        self.n_checks += 1
        d.qpos[:N_ARM] = q
        self.fingers.set_qpos(d.qpos, width_mm)
        carried = carried_T_tcp_part is not None
        if carried:
            mujoco.mj_kinematics(m, d)
            sid = m.site(tcp_site).id
            T_tcp = np.eye(4)
            T_tcp[:3, :3] = d.site_xmat[sid].reshape(3, 3)
            T_tcp[:3, 3] = d.site_xpos[sid]
            self.set_part(T_tcp @ carried_T_tcp_part)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_collision(m, d)

        worst = None
        for c in d.contact[:d.ncon]:
            hit = self._classify(c, env_clearance, part_tol, ignore_part, ignore_pad_contact, carried,
                                 carried_clearance)
            if hit is not None and (worst is None or hit.dist_mm < worst.dist_mm):
                worst = hit
        return worst

    def _classify(self, c, env_clearance, part_tol, ignore_part, ignore_pad_contact, carried, carried_clearance):
        m = self.model
        g1, g2 = c.geom1, c.geom2
        b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
        r1, r2 = self.robot_body[b1], self.robot_body[b2]
        dist = float(c.dist)
        hit = None

        if r1 and r2:
            if not (self.gripper_body[b1] and self.gripper_body[b2]) and dist < -SELF_TOL_M:
                hit = 'self', g1, g2, False
        elif r1 or r2:
            rg, og = (g1, g2) if r1 else (g2, g1)
            rb, ob = m.geom_bodyid[rg], m.geom_bodyid[og]
            if ob == self.part_bid:
                if carried:
                    if not self.gripper_body[rb] and dist < -SELF_TOL_M:
                        hit = 'part', rg, og, False
                elif not ignore_part and dist < -part_tol:
                    pad = rg in self.pad_geoms
                    if not (pad and ignore_pad_contact):
                        hit = 'part', rg, og, pad
            elif rb not in self.base_bodies and dist < env_clearance - SELF_TOL_M * (env_clearance == 0.0):
                hit = 'environment', rg, og, False
        elif carried and self.part_bid in (b1, b2):
            pg, og = (g1, g2) if b1 == self.part_bid else (g2, g1)
            if dist < carried_clearance - SELF_TOL_M * (carried_clearance == 0.0):
                hit = 'carried part', pg, og, False
        if hit is None:
            return None
        kind, a, b, pad = hit
        return Hit(kind, self._name(a), self._name(b), dist * 1000.0, pad)
