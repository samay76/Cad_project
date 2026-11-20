"""
cell_scene.py - The cell without a part, for the web app's teaching view: table, tray, EC66 at its home pose with the
Robotiq 2F-85, in the same format as the live view (web_view.py). The fixture is drawn by the web app itself.

  python -m cell.cell_scene            # writes var/cache/webview/cell/<config hash>/{scene.json, frames.bin}, prints the folder

Cached per configuration (calibration, home pose, tray), so it is built once (a few seconds) and then reused.
T_view maps the sim world into the frame of a part seated on the fixture (part turned 0 deg), like a live run.
"""
import hashlib
import json
import os
import sys

import mujoco
import numpy as np

from .build_robot import build
from .cameras import inspection_camera
from .cad_pick_place import robot_base_offset, FingerMap, tray_half_for
from .frames import inv, make_T
from .paths import CACHE, load_config
from .pick_station import from_config as station_from_config, seated_pose
from .web_view import WebView

ROBOT, GRIPPER = 'ec66', '2f85'
VERSION = 2   # 2: inspection camera


def build_cell(cfg=None):
    cfg = cfg or load_config()
    key = hashlib.sha1((json.dumps(cfg, sort_keys=True) + f'v{VERSION}').encode()).hexdigest()[:16]
    out = os.path.join(CACHE, 'webview', 'cell', key)
    if os.path.exists(os.path.join(out, 'scene.json')):
        return out
    os.makedirs(out, exist_ok=True)
    # STEP files first: OpenCascade crashes once MuJoCo has built a model in the same process
    info, placement = station_from_config(cfg)
    coupling = float(cfg['gripper_coupling_mm'][ROBOT][GRIPPER])
    model = build(robot=ROBOT, coupling_mm=coupling, base_offset_mm=tuple(robot_base_offset(cfg, ROBOT)),
                  tray_xy=cfg['place_xy'], tray_half=tray_half_for(np.zeros((1, 3)), np.zeros(3), cfg),
                  fine_fingers=True, inspection_camera=inspection_camera(cfg))
    data = mujoco.MjData(model)
    home = np.radians(np.asarray(cfg['pick_motion']['home_joints_deg'][ROBOT], dtype=float))
    FingerMap(model, home).set_qpos(data.qpos, 85.0)      # open, as at home
    data.qpos[:6] = home
    mujoco.mj_forward(model, data)
    if placement is not None:
        pos, quat = seated_pose(dict(placement, part_yaw_deg=0.0))
        T_part = make_T(pos, quat)
    else:
        T_part = make_T([*cfg['pick_xy'], 0.0], [1, 0, 0, 0])
    look = {k: v for k, v in (cfg.get('web_view') or {}).items() if k.endswith('_color')}
    WebView(model, data, out, inv(T_part), label='cell', look=look).close()
    return out


def main():
    print(build_cell())
    return 0


if __name__ == '__main__':
    sys.exit(main())
