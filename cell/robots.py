"""
robots.py - The robot arm of the cell: Elite EC66 (ec66/ec66.xml, from Elite's official URDF,
github.com/Elite-Robots/ROS).

The arm is a MuJoCo model whose first 6 joints are the arm joints (ik.N_ARM). The gripper is attached at
`flange_pos/flange_quat` on `flange_body` (+Z out of the tool flange), see frames.py.
"""
import os
from dataclasses import dataclass

import numpy as np
from .paths import MODELS



@dataclass(frozen=True)
class RobotSpec:
    id: str
    label: str
    xml: str                      # relative to models/
    flange_body: str              # body that carries the tool flange
    flange_pos: tuple             # tool flange in flange_body (m)
    flange_quat: tuple            # (w, x, y, z), +Z out of the flange
    home: tuple                   # home arm posture (rad), TCP above the work area pointing down
    base_bodies: tuple            # bodies that may touch the table (ignored by the env collision check)
    kp: tuple = (2000, 2000, 1500, 400, 400, 200)   # joint position gains (build_robot.py)
    kv: tuple = (200, 200, 150, 40, 40, 20)         # joint damping / velocity gains

    @property
    def xml_path(self):
        return os.path.join(MODELS, self.xml)

    @property
    def home_qpos(self):
        return np.array(self.home, dtype=np.float64)


ROBOTS = {
    'ec66': RobotSpec(
        id='ec66', label='Elite EC66', xml='ec66/ec66.xml',
        flange_body='link6', flange_pos=(0.0, -0.089, 0.0), flange_quat=(0.0, 0.0, -0.707107, 0.707107),
        # Default elbow-up posture; the real home is config/cell.json pick_motion.home_joints_deg.ec66.
        home=(0.045, -1.284, 1.705, -2.022, 1.569, 0.045),
        base_bodies=('base_link', 'link1'),
        # Lower gains than a heavy arm: higher ones follow the corners of the piecewise-linear joint path
        # sharply and shake the part loose (2-3x the slip).
        kp=(600, 600, 450, 120, 120, 60),
        kv=(60, 60, 45, 12, 12, 6)),
}

DEFAULT_ROBOT = 'ec66'


def get_robot(robot_id):
    rid = robot_id or DEFAULT_ROBOT
    if rid not in ROBOTS:
        raise ValueError(f"Unknown robot '{rid}' (known: {', '.join(ROBOTS)})")
    return ROBOTS[rid]
