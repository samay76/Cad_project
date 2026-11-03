"""
robots.py - The robot arms the simulator can use.

Each arm is a MuJoCo model whose first 6 joints are the arm joints (ik.N_ARM). The gripper is
attached at `flange_pos/flange_quat` on `flange_body` (+Z out of the tool flange), so the gripper and
TCP conventions (frames.py) are the same for every arm.

  cs66 : Elite CS66 (cs66/cs66.xml), the original arm - default everywhere.
  ec66 : Elite EC66 (ec66/ec66.xml), from Elite's official URDF (github.com/Elite-Robots/ROS).
"""
import os
from dataclasses import dataclass

import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


@dataclass(frozen=True)
class RobotSpec:
    id: str
    label: str
    xml: str                      # relative to jenga_mujoco/
    flange_body: str              # body that carries the tool flange
    flange_pos: tuple             # tool flange in flange_body (m)
    flange_quat: tuple            # (w, x, y, z), +Z out of the flange
    home: tuple                   # home arm posture (rad), TCP above the work area pointing down
    base_bodies: tuple            # bodies that may touch the table (ignored by the env collision check)
    kp: tuple = (2000, 2000, 1500, 400, 400, 200)   # joint position gains (build_robot.py)
    kv: tuple = (200, 200, 150, 40, 40, 20)         # joint damping / velocity gains

    @property
    def xml_path(self):
        return os.path.join(BASE_DIR, self.xml)

    @property
    def home_qpos(self):
        return np.array(self.home, dtype=np.float64)


ROBOTS = {
    'cs66': RobotSpec(
        id='cs66', label='Elite CS66', xml='cs66/cs66.xml',
        flange_body='wrist_3_link', flange_pos=(0.0, 0.0, 0.0), flange_quat=(1.0, 0.0, 0.0, 0.0),
        home=(3.14159, -1.2, 1.8, -2.2, -1.57, 0.0),
        base_bodies=('base_link', 'shoulder_link')),
    'ec66': RobotSpec(
        id='ec66', label='Elite EC66', xml='ec66/ec66.xml',
        flange_body='link6', flange_pos=(0.0, -0.089, 0.0), flange_quat=(0.0, 0.0, -0.707107, 0.707107),
        # Elbow-up IK solution that puts the gripper TCP where the CS66 home puts it.
        home=(0.045, -1.284, 1.705, -2.022, 1.569, 0.045),
        base_bodies=('base_link', 'link1'),
        # The EC66 is ~3x lighter than the CS66. With the CS66 gains it follows the corners of the
        # piecewise-linear joint path sharply and shakes the part loose (2-3x the slip); 0.3x gives
        # the CS66's slip on the same grasps.
        kp=(600, 600, 450, 120, 120, 60),
        kv=(60, 60, 45, 12, 12, 6)),
}

DEFAULT_ROBOT = 'cs66'


def get_robot(robot_id):
    rid = robot_id or DEFAULT_ROBOT
    if rid not in ROBOTS:
        raise ValueError(f"Unknown robot '{rid}' (known: {', '.join(ROBOTS)})")
    return ROBOTS[rid]
