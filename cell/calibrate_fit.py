"""
calibrate_fit.py - Calibrates the robot base offset AND the gripper length from several fingertip touches.

One touch with the gripper pointing straight down cannot tell "the robot base is higher" from "the gripper is
shorter": both move the fingertips up. Tilted picks then miss along the gripper's axis. With one more touch from
another direction (the gripper level, fingertips flat against a face of the scale) both can be solved:

  unknowns: robot_base_offset_mm (x, y, z) and gripper_coupling_mm (the gripper's distance from the flange)

Each touch is recorded from the robot's joints (read only, the robot is not moved):

  python -m cell.calibrate_fit record top                    # closed fingertips on the centre of the scale top,
                                                             # gripper straight down
  python -m cell.calibrate_fit record face X Y Z NX NY NZ    # closed fingertips flat against the scale at the point
                                                             # (scale frame, mm) on the face with that normal
  python -m cell.calibrate_fit list | clear
  python -m cell.calibrate_fit solve [--write]               # fit, show the residual of every touch, save

Add --joints J1 .. J6 (degrees) to record a pose without the robot. Touches go to var/calibration/touches.json and
are kept relative to the joints, so they stay valid when the config changes.
"""
import argparse
import json
import os
import re
import sys

import mujoco
import numpy as np

from .build_robot import build
from .cad_pick_place import FingerMap, robot_base_offset
from .paths import CONFIG_FILE, DATA, load_config
from .calibrate_robot import SCALE_TOP_MM

TOUCHES = os.path.join(DATA, 'calibration', 'touches.json')
ROBOT, GRIPPER = 'ec66', '2f85'


def scale_world(cfg, p_mm, n=None):
    """World position (m) of a point of the scale (cad/scale.step frame, mm) where the sim bolts it, and a normal."""
    from .paths import repo_path
    from .pick_station import load, place, seated_pose
    ps = cfg['pick_station']
    info = load(repo_path(ps['file']))
    placement = dict(place(info, ps.get('near_xy', cfg['pick_xy']), ps.get('yaw_deg', 0.0)),
                     bottom_z_mm=info['bottom_z_mm'])
    pos, quat = seated_pose(placement)
    yaw = 2 * np.arctan2(quat[3], quat[0])
    R = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    p = R @ (np.asarray(p_mm, dtype=float) / 1000.0) + np.asarray(pos)
    return p, (None if n is None else R @ (np.asarray(n, dtype=float) / np.linalg.norm(n)))


def tip_and_axis(cfg, deg):
    """Sim fingertip (m, world) of the closed gripper at these joints, and the gripper axis (unit, world).
    Fingertip = centre of the finger surface that reaches furthest along the gripper axis."""
    coupling = float(cfg['gripper_coupling_mm'][ROBOT][GRIPPER])
    model = build(robot=ROBOT, coupling_mm=coupling, base_offset_mm=tuple(robot_base_offset(cfg, ROBOT)))
    data = mujoco.MjData(model)
    q = np.radians(np.asarray(deg, dtype=float))
    FingerMap(model, q).set_qpos(data.qpos, 0.0)        # fingers closed
    data.qpos[:6] = q
    mujoco.mj_forward(model, data)
    axis = data.site_xmat[model.site('g_tcp').id].reshape(3, 3)[:, 2]
    pts = []
    for g in range(model.ngeom):
        if (not model.body(model.geom_bodyid[g]).name.startswith('g_') or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH
                or model.geom_group[g] >= 3 or model.geom_rgba[g][3] == 0):   # the visible finger shape only
            continue
        m = model.geom_dataid[g]
        a, n = model.mesh_vertadr[m], model.mesh_vertnum[m]
        pts.append(model.mesh_vert[a:a + n] @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g])
    pts = np.vstack(pts)
    s = pts @ axis
    return pts[s > s.max() - 0.001].mean(axis=0), axis


def read_joints(args):
    if args.joints:
        return [float(v) for v in args.joints]
    from .real_robot import connect
    robot = connect(args.ip, args.src)
    deg = robot.joints()
    print(f"Robot joints (deg): {' '.join(f'{v:8.2f}' for v in deg)}   mode {robot.mode()}")
    robot.close()
    return deg


def load_touches():
    try:
        with open(TOUCHES) as f:
            return json.load(f)
    except FileNotFoundError:
        return []


def save_touches(touches):
    os.makedirs(os.path.dirname(TOUCHES), exist_ok=True)
    with open(TOUCHES, 'w') as f:
        json.dump(touches, f, indent=1)


def equations(cfg, touch):
    """Rows A (n x 4: base x, y, z, coupling) and b (n) of the linear model, plus the current error (mm)."""
    tip, axis = tip_and_axis(cfg, touch['joints'])
    if touch['kind'] == 'top':
        target, _ = scale_world(cfg, SCALE_TOP_MM)
        A = np.hstack([np.eye(3), axis[:, None]])
        b = target - tip
        return A, b, b * 1000.0
    target, n = scale_world(cfg, touch['point'], touch['normal'])
    A = np.array([[*n, n @ axis]])
    b = np.array([n @ (target - tip)])
    return A, b, b * 1000.0


def solve(cfg, touches):
    rows = [equations(cfg, t) for t in touches]
    A = np.vstack([r[0] for r in rows])
    b = np.concatenate([r[1] for r in rows])
    if np.linalg.matrix_rank(A, tol=1e-3) < 4:
        raise SystemExit('Not enough different touches: record one "top" (gripper down) and one "face" (gripper level).')
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    return x * 1000.0, rows, A, b


def write(cfg, d):
    s = open(CONFIG_FILE).read()
    off = np.array(robot_base_offset(cfg, ROBOT)) + d[:3]
    coupling = float(cfg['gripper_coupling_mm'][ROBOT][GRIPPER]) + d[3]
    old_off = json.dumps(cfg['robot_base_offset_mm'][ROBOT])
    s2, n1 = re.subn(r'("robot_base_offset_mm"[^}]*?"ec66":\s*)' + re.escape(old_off),
                     lambda m: m.group(1) + f'[{off[0]:.1f}, {off[1]:.1f}, {off[2]:.1f}]', s, count=1, flags=re.S)
    s3, n2 = re.subn(r'("gripper_coupling_mm"[^}]*?"2f85":\s*)[-0-9.]+',
                     lambda m: m.group(1) + f'{coupling:.1f}', s2, count=1, flags=re.S)
    if n1 != 1 or n2 != 1:
        raise SystemExit('Could not find the keys in config/cell.json: set them by hand.')
    json.loads(s3)
    open(CONFIG_FILE, 'w').write(s3)
    print(f'Saved: robot_base_offset_mm.ec66 = [{off[0]:.1f}, {off[1]:.1f}, {off[2]:.1f}], '
          f'gripper_coupling_mm.ec66.2f85 = {coupling:.1f}. Restart the server and plan again.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('command', choices=['record', 'list', 'clear', 'solve'])
    ap.add_argument('kind', nargs='?', choices=['top', 'face'])
    ap.add_argument('point', nargs='*', type=float, help='face: X Y Z NX NY NZ (scale frame, mm and normal)')
    ap.add_argument('--joints', nargs=6, type=float)
    ap.add_argument('--ip', default='192.168.1.200')
    ap.add_argument('--src')
    ap.add_argument('--write', action='store_true')
    args = ap.parse_args()
    cfg = load_config()
    touches = load_touches()

    if args.command == 'clear':
        save_touches([])
        print('All touches removed.')
        return 0
    if args.command == 'list':
        for i, t in enumerate(touches):
            print(i, t['kind'], t.get('point', ''), t['joints'])
        return 0
    if args.command == 'record':
        if args.kind is None or (args.kind == 'face' and len(args.point) != 6):
            raise SystemExit('record top | record face X Y Z NX NY NZ')
        t = {'kind': args.kind, 'joints': read_joints(args)}
        if args.kind == 'face':
            t['point'], t['normal'] = args.point[:3], args.point[3:]
        _, _, err = equations(cfg, t)
        touches.append(t)
        save_touches(touches)
        what = 'dx, dy, dz' if t['kind'] == 'top' else 'distance to the face along its normal'
        print(f"Recorded touch {len(touches)} ({t['kind']}). Error now (real - sim), {what}: "
              f"{', '.join(f'{v:+.1f}' for v in err)} mm")
        return 0

    d, rows, A, b = solve(cfg, touches)
    print(f'{len(touches)} touches. Correction: base dx={d[0]:+.1f} dy={d[1]:+.1f} dz={d[2]:+.1f} mm, '
          f'gripper {d[3]:+.1f} mm further from the flange')
    for i, (t, (Ai, bi, err)) in enumerate(zip(touches, rows)):
        after = (bi - Ai @ (d / 1000.0)) * 1000.0
        print(f"  touch {i + 1} ({t['kind']}): before {', '.join(f'{v:+.1f}' for v in err)} mm  ->  after "
              f"{', '.join(f'{v:+.1f}' for v in after)} mm")
    if np.any(np.abs(d[:3]) > 30) or abs(d[3]) > 25:
        print('The correction is large: check the touches (gripper closed, fingertips really on the points).')
        return 2
    if args.write:
        write(cfg, d)
    else:
        print('Add --write to save it.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
