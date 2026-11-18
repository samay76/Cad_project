#!/usr/bin/env python3
"""
calibrate_robot.py - Measure where the real robot really is relative to the table, so the sim (and the plans made in
it) put the real gripper exactly where the sim shows it.

  1. Close the gripper fully:   python calibrate_robot.py --close-gripper
  2. With the teach pendant, jog the robot so the CLOSED FINGERTIPS just touch the table at the CENTRE of a big
     (26 mm) table hole - by default hole (15, 0), the pick hole. Gripper pointing straight down.
  3. python calibrate_robot.py                       # reads the joints (read-only), prints the error
     python calibrate_robot.py --hole 16 12          # another hole (i along x, j along y, 0..16)
     python calibrate_robot.py --write               # also saves the corrected robot_base_offset_mm
     python calibrate_robot.py --joints 179 -80 ...  # use these joint angles (deg) instead of reading the robot

It compares the hole (where the fingertips really are) with where the sim puts the fingertips for those joints, and
corrects sim_config.json robot_base_offset_mm by the difference. Repeat at a second hole far from the first to check:
after --write, both should show an error of a millimetre or two.

Reads only: it never moves the robot or the gripper.
"""
import argparse
import json
import os
import sys

import mujoco
import numpy as np

from build_robot import build
from cad_pick_place import FingerMap, robot_base_offset

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(BASE_DIR, 'sim_config.json')
BIG_HOLE_ORIGIN = (-0.1004, -0.3402)   # big hole (0, 0) in the sim world frame (m), as show_locations.py
BIG_HOLE_PITCH = 0.040


def fingertip(model, data):
    """Centre (x, y) of the lowest part of the closed fingers and its height z (m, sim world frame)."""
    pts = []
    for g in range(model.ngeom):
        body = model.body(model.geom_bodyid[g]).name
        if not body.startswith('g_') or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        m = model.geom_dataid[g]
        a, n = model.mesh_vertadr[m], model.mesh_vertnum[m]
        v = model.mesh_vert[a:a + n] @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        pts.append(v)
    pts = np.vstack(pts)
    z = pts[:, 2].min()
    low = pts[pts[:, 2] < z + 0.001]
    return np.array([low[:, 0].mean(), low[:, 1].mean(), z])


def close_gripper(gcfg):
    """Closes the real gripper fully (step 1). Moves only the fingers."""
    import time
    from real_gripper import RealGripper
    g = RealGripper(gcfg.get('port', '/dev/ttyUSB0'), int(gcfg.get('baud', 115200)), int(gcfg.get('slave', 9)))
    try:
        if not g.activated():
            print('Activating the gripper (the fingers close and open once)...')
            g.activate()
        g.command(255, int(gcfg.get('speed', 64)), int(gcfg.get('force', 50)))
        t0 = time.time()
        while not g.done(255)[0]:
            if time.time() - t0 > 8.0:
                raise RuntimeError('the gripper did not close within 8 s')
            time.sleep(0.05)
        print(f'Gripper closed (position {g.position()}/255).')
    finally:
        g.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hole', nargs=2, type=int, default=[15, 0], help='big table hole the fingertips touch (i j)')
    ap.add_argument('--ip', default='192.168.1.200')
    ap.add_argument('--src', help='local IP to connect from (found automatically if omitted)')
    ap.add_argument('--joints', nargs=6, type=float, help='joint angles in degrees instead of reading the robot')
    ap.add_argument('--write', action='store_true', help='save the corrected robot_base_offset_mm')
    ap.add_argument('--close-gripper', action='store_true',
                    help='only close the real 2F-85 fully (sim_config.json real_gripper port) and exit')
    args = ap.parse_args()

    cfg = json.load(open(CONFIG))
    if args.close_gripper:
        close_gripper(cfg.get('real_gripper') or {})
        return
    robot_id, gripper = 'ec66', '2f85'
    offset = np.array(robot_base_offset(cfg, robot_id), dtype=float)
    coupling = float((cfg.get('gripper_coupling_mm') or {}).get(robot_id, {}).get(gripper, 0.0))

    if args.joints:
        deg = np.array(args.joints)
    else:
        from real_robot import connect
        robot = connect(args.ip, args.src)
        deg = np.array(robot.joints())
        print('Robot joints (deg): ' + ' '.join(f'{v:8.2f}' for v in deg) + f'   mode {robot.mode()}')
        robot.close()

    model = build(robot=robot_id,
                  coupling_mm=coupling, base_offset_mm=tuple(offset))
    data = mujoco.MjData(model)
    q = np.radians(deg)
    FingerMap(model, q).set_qpos(data.qpos, 0.0)          # fingers closed
    data.qpos[:6] = q
    mujoco.mj_forward(model, data)
    tip = fingertip(model, data)
    down = -data.site_xmat[model.site('g_tcp').id].reshape(3, 3)[:, 2] @ np.array([0, 0, -1.0])

    i, j = args.hole
    hole = np.array([BIG_HOLE_ORIGIN[0] + i * BIG_HOLE_PITCH, BIG_HOLE_ORIGIN[1] + j * BIG_HOLE_PITCH, 0.0])
    err = (hole - tip) * 1000.0
    new = offset + err
    print(f'\nCurrent robot_base_offset_mm: [{offset[0]:.1f}, {offset[1]:.1f}, {offset[2]:.1f}]   (coupling {coupling:.0f} mm)')
    print(f'Real fingertips (hole ({i}, {j}) centre, on the table): x={hole[0] * 1000:7.1f} y={hole[1] * 1000:7.1f} z=  0.0 mm')
    print(f'Sim fingertips for these joints:                     x={tip[0] * 1000:7.1f} y={tip[1] * 1000:7.1f} z={tip[2] * 1000:5.1f} mm')
    if abs(down) < 0.97:
        print(f'  (the gripper is tilted {np.degrees(np.arccos(min(1.0, abs(down)))):.0f} deg from straight down: '
              'point it straight down for a clean measurement)')
    print(f'Error (real - sim): dx={err[0]:+.1f}  dy={err[1]:+.1f}  dz={err[2]:+.1f} mm')
    if np.linalg.norm(err) < 1.5:
        print('Within 1.5 mm: the sim matches the real robot here. Nothing to change.')
        return 0
    print(f'Corrected robot_base_offset_mm: [{new[0]:.1f}, {new[1]:.1f}, {new[2]:.1f}]')
    if np.linalg.norm(err[:2]) > 15 or abs(err[2]) > 30:
        print('  The error is large: check that the fingertips are on the CENTRE of the right hole, the gripper is '
              'closed and points straight down. Not writing it automatically.' if args.write else
              '  The error is large: check the hole number and that the gripper is closed and points straight down.')
        if args.write:
            return 2
    if args.write:
        s = open(CONFIG).read()
        old = json.dumps(cfg['robot_base_offset_mm'][robot_id])
        line = f'"{robot_id}": [{new[0]:.1f}, {new[1]:.1f}, {new[2]:.1f}]'
        key = f'"{robot_id}": {old}'
        if key not in s:
            print(f'Could not find {key} in sim_config.json; set robot_base_offset_mm.{robot_id} by hand.')
            return 2
        start = s.index('"robot_base_offset_mm"')
        k = s.index(key, start)
        s = s[:k] + line + s[k + len(key):]
        open(CONFIG, 'w').write(s)
        print('Saved to sim_config.json. Re-plan in the web app (Plan to Points) before Sim / Real Sim.')
    else:
        print('Add --write to save it (then re-plan in the web app).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
