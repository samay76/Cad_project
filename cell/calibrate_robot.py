#!/usr/bin/env python3
"""
calibrate_robot.py - Measure where the real robot really is relative to the table, so the sim (and the plans made in
it) put the real gripper exactly where the sim shows it.

  1. Close the gripper fully:   python -m cell.calibrate_robot --close-gripper
  2. With the teach pendant, jog the robot so the CLOSED FINGERTIPS just touch the table at the CENTRE of a big
     (26 mm) table hole - by default hole (15, 0), the pick hole. Gripper pointing straight down.
  3. python -m cell.calibrate_robot                       # reads the joints (read-only), prints the error
     python -m cell.calibrate_robot --hole 16 12          # another hole (i along x, j along y, 0..16)
     python -m cell.calibrate_robot --write               # also saves the corrected robot_base_offset_mm
     python -m cell.calibrate_robot --joints 179 -80 ...  # use these joint angles (deg) instead of reading the robot

It compares the hole (where the fingertips really are) with where the sim puts the fingertips for those joints, and
corrects config/cell.json robot_base_offset_mm by the difference. Repeat at a second hole far from the first to check:
after --write, both should show an error of a millimetre or two.

Reads only: it never moves the robot or the gripper.
"""
import argparse
import json
import sys

import mujoco
import numpy as np

from .build_robot import build
from .cad_pick_place import FingerMap, robot_base_offset
from .paths import CONFIG_FILE

CONFIG = CONFIG_FILE
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
    from .real_gripper import RealGripper
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


FIXTURE_TOP_MM = (0.0, 0.0, 25.0)   # centre of the top face of the locating pin, cad/Fixture2.step frame (yoke off)


def fixture_top_world(cfg):
    """World position (m) of the top of the fixture's locating pin, where the sim bolts the fixture."""
    from .paths import repo_path
    from .pick_station import load, place, seated_pose
    ps = cfg['pick_station']
    info = load(repo_path(ps['file']))
    placement = dict(place(info, ps.get('near_xy', cfg['pick_xy']), ps.get('yaw_deg', 0.0)),
                     bottom_z_mm=info['bottom_z_mm'])
    pos, quat = seated_pose(placement)
    yaw = 2 * np.arctan2(quat[3], quat[0])
    c, s = np.cos(yaw), np.sin(yaw)
    x, y, z = (v / 1000.0 for v in FIXTURE_TOP_MM)
    return np.array([pos[0] + c * x - s * y, pos[1] + s * x + c * y, pos[2] + z])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hole', nargs=2, type=int, default=[15, 0], help='big table hole the fingertips touch (i j)')
    ap.add_argument('--fixture-top', action='store_true',
                    help='the fingertips touch the centre of the top of the fixture\'s locating pin (cad/Fixture2.step, '
                         'yoke removed) instead of a table hole: calibrates right where the touch / grip tests work')
    ap.add_argument('--ip', default='192.168.1.200')
    ap.add_argument('--src', help='local IP to connect from (found automatically if omitted)')
    ap.add_argument('--joints', nargs=6, type=float, help='joint angles in degrees instead of reading the robot')
    ap.add_argument('--write', action='store_true', help='save the corrected robot_base_offset_mm')
    ap.add_argument('--close-gripper', action='store_true',
                    help='only close the real 2F-85 fully (config/cell.json real_gripper port) and exit')
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
        from .real_robot import connect
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

    if args.fixture_top:
        hole, where = fixture_top_world(cfg), 'centre of the top of the fixture pin'
    else:
        i, j = args.hole
        hole = np.array([BIG_HOLE_ORIGIN[0] + i * BIG_HOLE_PITCH, BIG_HOLE_ORIGIN[1] + j * BIG_HOLE_PITCH, 0.0])
        where = f'hole ({i}, {j}) centre, on the table'
    err = (hole - tip) * 1000.0
    new = offset + err
    print(f'\nCurrent robot_base_offset_mm: [{offset[0]:.1f}, {offset[1]:.1f}, {offset[2]:.1f}]   (coupling {coupling:.0f} mm)')
    print(f'Real fingertips ({where}): x={hole[0] * 1000:7.1f} y={hole[1] * 1000:7.1f} z={hole[2] * 1000:6.1f} mm')
    print(f'Sim fingertips for these joints:                     x={tip[0] * 1000:7.1f} y={tip[1] * 1000:7.1f} z={tip[2] * 1000:5.1f} mm')
    if abs(down) < 0.97:
        print(f'  (the gripper is tilted {np.degrees(np.arccos(min(1.0, abs(down)))):.0f} deg from straight down: '
              'point it straight down for a clean measurement)')
    print(f'Error (real - sim): dx={err[0]:+.1f}  dy={err[1]:+.1f}  dz={err[2]:+.1f} mm')
    if np.linalg.norm(err) < 1.5:
        print('Within 1.5 mm: the sim matches the real robot here. Nothing to change.')
        return 0
    print(f'Corrected robot_base_offset_mm: [{new[0]:.1f}, {new[1]:.1f}, {new[2]:.1f}]')
    if np.linalg.norm(err[:2]) > (40 if args.fixture_top else 15) or abs(err[2]) > 40:
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
            print(f'Could not find {key} in config/cell.json; set robot_base_offset_mm.{robot_id} by hand.')
            return 2
        start = s.index('"robot_base_offset_mm"')
        k = s.index(key, start)
        s = s[:k] + line + s[k + len(key):]
        open(CONFIG, 'w').write(s)
        print('Saved to config/cell.json. Re-plan in the web app (Plan to Points) before Sim / Real Sim.')
    else:
        print('Add --write to save it (then re-plan in the web app).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
