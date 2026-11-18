#!/usr/bin/env python3
"""
show_locations.py - Where the sim puts the part (pick) and where it drops it (place), so you can set up the real table
the same way.

  python show_locations.py                    # the newest plan made in the web app (runs/*/plan.json), grasp 1
  python show_locations.py --candidate 2      # grasp 2 of that plan
  python show_locations.py --run runs/<id>    # a specific run
  python show_locations.py --view             # also open MuJoCo: the part at the pick, a dot at the drop

The pick always follows the current sim_config.json (pick_xy); the drop comes from the newest plan when that plan
was made for the same pick (otherwise it says to re-plan).

Coordinates are the sim's world frame in mm: x/y from the robot base axis, z up from the table top. The robot looks
along +x over the table. Positions are also given relative to the nearest BIG (26 mm) table hole: holes (i, j) form a
17 x 17 grid with 40 mm pitch; i counts along x (0 = nearest the robot's back, 16 = far end), j along y (0 = right
side, 16 = left side, seen from behind the robot looking along +x). Corner B = (16, 16) far left, D = (0, 16) near left.

Reads only: it never connects to the robot or the gripper.
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
BIG_HOLE_ORIGIN = (-0.1004, -0.3402)   # big 26 mm hole (0, 0), world frame (m) - as ec66_sim_standalone/ec66_sim.py
BIG_HOLE_PITCH = 0.040
BIG_HOLE_N = 17


def nearest_hole(x, y):
    i = int(round((x - BIG_HOLE_ORIGIN[0]) / BIG_HOLE_PITCH))
    j = int(round((y - BIG_HOLE_ORIGIN[1]) / BIG_HOLE_PITCH))
    i, j = min(max(i, 0), BIG_HOLE_N - 1), min(max(j, 0), BIG_HOLE_N - 1)
    hx, hy = BIG_HOLE_ORIGIN[0] + i * BIG_HOLE_PITCH, BIG_HOLE_ORIGIN[1] + j * BIG_HOLE_PITCH
    return i, j, (x - hx) * 1000, (y - hy) * 1000


def describe(title, verts_w, T):
    lo, hi = verts_w.min(0) * 1000, verts_w.max(0) * 1000
    c = (lo + hi) / 2
    yaw = math.degrees(math.atan2(T[1, 0], T[0, 0]))
    i, j, dx, dy = nearest_hole(c[0] / 1000, c[1] / 1000)
    print(f"\n{title}")
    print(f"  footprint centre   x={c[0]:7.1f}  y={c[1]:7.1f} mm   (nearest big hole ({i}, {j}), "
          f"centre is {dx:+.0f} mm in x, {dy:+.0f} mm in y from it)")
    print(f"  footprint size     {hi[0] - lo[0]:.1f} (x) x {hi[1] - lo[1]:.1f} (y) mm, height {hi[2] - lo[2]:.1f} mm")
    print(f"  footprint corners  x {lo[0]:.1f} .. {hi[0]:.1f}   y {lo[1]:.1f} .. {hi[1]:.1f} mm")
    print(f"  bottom / top       z {lo[2]:.1f} / {hi[2]:.1f} mm above the table top")
    print(f"  part yaw           {yaw:.1f} deg about the vertical (CAD x axis vs table x axis)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', help='run folder (default: the newest runs/*/ with a plan.json)')
    ap.add_argument('--candidate', type=int, default=1, help='grasp number as shown in the web app (1, 2, 3)')
    ap.add_argument('--view', action='store_true', help='open MuJoCo with the part at the pick and at the drop')
    args = ap.parse_args()

    if args.run:
        run = args.run
    else:
        plans = glob.glob(os.path.join(BASE_DIR, 'runs', '*', 'plan.json'))
        if not plans:
            sys.exit('No plan found: press Plan to Points in the web app first.')
        run = os.path.dirname(max(plans, key=os.path.getmtime))
    doc = json.load(open(os.path.join(run, 'plan.json')))
    if not 1 <= args.candidate <= len(doc['plans']):
        sys.exit(f"The plan has {len(doc['plans'])} grasps.")
    plan = doc['plans'][args.candidate - 1]
    print(f"Run {os.path.basename(run.rstrip('/'))}, grasp {args.candidate}: {plan['mode']} grip, "
          f"{plan['widthMm']:.1f} mm, robot {doc['robotLabel']}, gripper {doc['gripperLabel']}")

    # The part's shape comes from the simulator scene (same code as the web app's planner).
    from auto_pick_place import AutoPickPlace, load_request
    from cad_pick_place import Status

    class A:
        request = os.path.join(run, 'request.json')
        step = click = normal = point1 = point2 = robot = gripper = None
    step, req, q_part = load_request(A)
    req = dict(req, robotId=doc['robot'], gripperId=doc['gripper'], tcpOffset=doc['tcpOffset'],
               preGraspDistance=plan['grasp']['preGraspDistance'], grasp=plan['grasp'])
    cfg = json.load(open(os.path.join(BASE_DIR, 'sim_config.json')))
    sim = AutoPickPlace(step, req, q_part, cfg, Status(None), headless=not args.view)
    verts = sim.part_verts

    # The pick comes from the CURRENT sim_config.json: the part is let settle on the table, as the planner does.
    T_pick = sim.settle()
    T_planned = np.array(doc['part']['T_world_part'])
    stale = (np.linalg.norm(T_pick[:3, 3] - T_planned[:3, 3]) > 0.005 or
             abs(np.trace(T_pick[:3, :3].T @ T_planned[:3, :3]) - 3) > 1e-3)
    T_drop = None if stale else np.array(plan['T_wp_placed'])
    describe('PICK - put the real part here (as it lies on the table in the sim, current sim_config.json):',
             verts @ T_pick[:3, :3].T + T_pick[:3, 3], T_pick)
    if stale:
        print("\n  NOTE: the newest plan was made for a different pick location (sim_config.json changed since).")
        print("        Press Plan to Points in the web app again before Sim / Real Sim. The drop pose below needs")
        print("        a new plan; the tray position is from the current settings.")
    else:
        describe('DROP - where the gripper lets go of it (it then falls into the tray):',
                 verts @ T_drop[:3, :3].T + T_drop[:3, 3], T_drop)
    if not stale and doc['part'].get('tippedDeg', 0) > 5:
        print(f"\n  NOTE: in the sim the part tipped over {doc['part']['tippedDeg']:.0f} deg when placed on the table: "
              "put the real one down the same way (see --view).")

    tray = True
    if tray:
        tx, ty = (v * 1000 for v in cfg['place_xy'])
        hx, hy = (v * 1000 for v in sim.tray_half)
        i, j, dx, dy = nearest_hole(tx / 1000, ty / 1000)
        print(f"\nTRAY (drop location)\n  centre x={tx:.1f} y={ty:.1f} mm (nearest big hole ({i}, {j}), {dx:+.0f} / {dy:+.0f} mm)"
              f", inside {2 * hx:.0f} x {2 * hy:.0f} mm: x {tx - hx:.1f} .. {tx + hx:.1f}, y {ty - hy:.1f} .. {ty + hy:.1f}")
    print(f"\n(pick_xy {cfg['pick_xy']} m and place_xy {cfg['place_xy']} m in sim_config.json set these positions.)")

    if args.view:
        import mujoco
        sim.open_viewer()
        sim._set_part_qpos(T_pick)
        sim.data.qpos[:6] = sim.home
        sim.data.qvel[:] = 0
        mujoco.mj_forward(sim.model, sim.data)
        if T_drop is not None:
            sim.show_path([T_drop[:3, 3]])
            sim.overlay('PART AT THE PICK', 'Yellow dot = where it is dropped. Close the window to quit.')
        else:
            sim.overlay('PART AT THE PICK (current settings)', 'Re-plan in the web app for the drop pose. '
                        'Close the window to quit.')
        while sim.viewer.is_running():
            sim.viewer.sync()
            time.sleep(1 / 30)


if __name__ == '__main__':
    main()
