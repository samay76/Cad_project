#!/usr/bin/env python3
"""
test_auto_pick_place.py - Acceptance test of the automatic pick & place, with full physics, no window.

For each case: plan from a clicked point, then execute the plan and require that the simulator reports
the part in the tray. Cases that must fail are required to fail at planning, with a useful reason.

  python test_auto_pick_place.py            # everything (a few minutes)
  python test_auto_pick_place.py box ring   # only the cases whose name contains one of these
  python test_auto_pick_place.py --snapshots out/    # keep a PNG per phase (needs MUJOCO_GL=egl)
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

BASE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(BASE)


def make_step(path, build):
    """Writes a STEP test part built with OpenCascade (mm)."""
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
    saved = os.dup(1)  # OpenCascade logs to the raw stdout
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1)
    try:
        writer = STEPControl_Writer()
        writer.Transfer(build(), STEPControl_AsIs)
        done = writer.Write(path)
    finally:
        os.dup2(saved, 1)
        os.close(saved)
        os.close(devnull)
    if done != IFSelect_RetDone:
        raise RuntimeError(f'Could not write {path}')


def box(dx, dy, dz):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    return lambda: BRepPrimAPI_MakeBox(float(dx), float(dy), float(dz)).Shape()


def tube_on_side(outer_r, inner_r, length):
    def build():
        from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
        from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
        axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
        return BRepAlgoAPI_Cut(BRepPrimAPI_MakeCylinder(axis, float(outer_r), float(length)).Shape(),
                               BRepPrimAPI_MakeCylinder(axis, float(inner_r), float(length)).Shape()).Shape()
    return build


def ring(outer_r, inner_r, height):
    def build():
        from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
        return BRepAlgoAPI_Cut(BRepPrimAPI_MakeCylinder(float(outer_r), float(height)).Shape(),
                               BRepPrimAPI_MakeCylinder(float(inner_r), float(height)).Shape()).Shape()
    return build


# Two marked points: one finger must end up on each.
# name, part, point 1 (x,y,z,nx,ny,nz), point 2, gripper, robot, expected result
POINT_CASES = [
    ('points_box_across_2f85', box(60, 30, 20), (30, 0, 10, 0, -1, 0), (30, 30, 10, 0, 1, 0), '2f85', 'cs66', 'tray:outside'),
    ('points_box_across_hande', box(60, 30, 20), (30, 0, 10, 0, -1, 0), (30, 30, 10, 0, 1, 0), 'hande', 'ec66', 'tray:outside'),
    ('points_box_off_centre', box(60, 30, 20), (15, 0, 10, 0, -1, 0), (15, 30, 10, 0, 1, 0), '2f85', 'cs66', 'tray:outside'),
    ('points_bore_2f85', ring(25.3, 17.85, 39), (17.85, 0, 30, -1, 0, 0), (-17.85, 0, 30, 1, 0, 0), '2f85', 'cs66', 'tray:inside'),
    ('points_bore_hande', ring(25.3, 17.85, 39), (0, 17.85, 30, 0, -1, 0), (0, -17.85, 30, 0, 1, 0), 'hande', 'ec66', 'tray:inside'),
    ('points_tube_across', ring(25.3, 17.85, 39), (25.3, 0, 20, 1, 0, 0), (-25.3, 0, 20, -1, 0, 0), '2f85', 'cs66', 'tray:outside'),
    # The points need not be lined up: only the surfaces must face each other, and both points must fit on the pads.
    ('points_box_not_lined_up', box(60, 30, 20), (20, 0, 10, 0, -1, 0), (35, 30, 10, 0, 1, 0), '2f85', 'cs66', 'tray:outside'),
    # One point on the inner wall and one on the outer wall, at different angles and heights.
    ('points_wall_not_lined_up', ring(25.3, 17.85, 39), (17.24, 4.62, 30, -0.966, -0.259, 0), (24.91, -4.39, 33, 0.985, -0.174, 0), '2f85', 'cs66', 'tray:outside'),
    # The whole sequence with the inspection camera: images must exist and the part must still reach the tray.
    ('points_camera_box_2f85', box(60, 30, 20), (30, 0, 10, 0, -1, 0), (30, 30, 10, 0, 1, 0), '2f85', 'cs66', 'tray:outside'),
    ('points_camera_box_hande', box(60, 30, 20), (30, 0, 10, 0, -1, 0), (30, 30, 10, 0, 1, 0), 'hande', 'ec66', 'tray:outside'),
    ('points_camera_bore_2f85', ring(25.3, 17.85, 39), (17.85, 0, 30, -1, 0, 0), (-17.85, 0, 30, 1, 0, 0), '2f85', 'cs66', 'tray:inside'),
    ('points_too_far_sideways', box(100, 30, 20), (5, 0, 10, 0, -1, 0), (95, 30, 10, 0, 1, 0), '2f85', 'cs66', 'plan-fails'),
    ('points_too_far_apart', box(100, 30, 20), (0, 15, 10, -1, 0, 0), (100, 15, 10, 1, 0, 0), '2f85', 'cs66', 'plan-fails'),
    ('points_same_side', box(60, 30, 20), (10, 15, 20, 0, 0, 1), (50, 15, 20, 0, 0, 1), '2f85', 'cs66', 'plan-fails'),
    ('points_not_opposite', box(60, 30, 20), (30, 0, 10, 0, -1, 0), (60, 15, 10, 1, 0, 0), '2f85', 'cs66', 'plan-fails'),
    ('points_too_far_from_opposite', ring(45, 30, 25), (45, 0, 12, 1, 0, 0), (-22.5, 38.97, 12, -0.5, 0.866, 0), '2f85', 'cs66', 'plan-fails'),
    ('points_bottom_and_top', box(60, 30, 20), (30, 15, 0, 0, 0, -1), (30, 15, 20, 0, 0, 1), '2f85', 'cs66', 'plan-fails'),
]
MAX_FINGER_ERROR_MM = 6.0   # a flat pad on a curved wall touches a little to the side of the marked point

# One selected point: the grasp is searched near it.
# name, part (file in the repo, or a builder), click point, surface normal, gripper, robot, expected result,
# and optionally grasp-planner settings. 'tray:inside' / 'tray:outside' also require that kind of grip.
CASES = [
    ('box_top_2f85_cs66', box(60, 30, 20), (30, 15, 20), (0, 0, 1), '2f85', 'cs66', 'tray'),
    ('box_side_hande_ec66', box(60, 30, 20), (30, 0, 10), (0, -1, 0), 'hande', 'ec66', 'tray'),
    ('part1_2f85_cs66', 'Cad model/Paart1.step', (0, 43, 0), (0, 1, 0), '2f85', 'cs66', 'tray'),
    ('part1_hande_cs66', 'Cad model/Paart1.step', (0, 43, 0), (0, 1, 0), 'hande', 'cs66', 'tray'),
    ('part2_2f85_ec66', 'Cad model/Part2.step', (0, 20, 30), (0, 0, 1), '2f85', 'ec66', 'tray'),
    # The only Hand-E-sized feature on this part is a thin flange, and the body blocks the gripper from it.
    ('part2_hande_blocked', 'Cad model/Part2.step', (0, 20, 30), (0, 0, 1), 'hande', 'cs66', 'plan-fails'),
    ('roller_2f85_cs66', 'skf_cam_roller_305703_c-2z_0.stp', (8.8, 0, 23.5), (0, 0, 1), '2f85', 'cs66', 'tray'),
    ('machined_2f85_cs66', 'public/samples/dm1-oc-214.stp', (16, -50, 40), (0, 0, 1), '2f85', 'cs66', 'tray'),
    # Clicking the wall of a bore: closed fingers go in and open. Never one finger in, one out.
    ('ring_inner_wall_2f85', ring(45, 30, 25), (30, 0, 12), (-1, 0, 0), '2f85', 'cs66', 'tray:inside'),
    ('ring_inner_wall_hande', ring(45, 30, 25), (30, 0, 12), (-1, 0, 0), 'hande', 'ec66', 'tray:inside'),
    ('tube_on_side_inner_2f85', tube_on_side(45, 30, 60), (30, 30, 0), (0, -1, 0), '2f85', 'cs66', 'tray:inside'),
    # With the Hand-E only a barely-fitting grip existed here, which slipped 37 mm. Refusing it is correct.
    ('tube_on_side_inner_hande', tube_on_side(45, 30, 60), (30, 30, 0), (0, -1, 0), 'hande', 'cs66', 'plan-fails'),
    # Clicking the outside of a tube too wide to span: the only valid grip is still from inside the bore.
    # Same proportions as a small sleeve (bore 35.7 mm): tight for the fingers.
    ('sleeve_bore_2f85', ring(25.3, 17.85, 39), (17.85, 0, 20), (-1, 0, 0), '2f85', 'cs66', 'tray:inside'),
    ('sleeve_bore_hande', ring(25.3, 17.85, 39), (17.85, 0, 20), (-1, 0, 0), 'hande', 'ec66', 'tray:inside'),
    # Clicking the outside of a tube the gripper can span: both fingers outside, across the diameter.
    ('sleeve_outer_2f85', ring(25.3, 17.85, 39), (25.3, 0, 20), (1, 0, 0), '2f85', 'cs66', 'tray:outside'),
    ('ring_outer_wall_2f85', ring(45, 30, 25), (45, 0, 12), (1, 0, 0), '2f85', 'cs66', 'tray:inside'),
    ('wide_ring_2f85', ring(70, 40, 25), (40, 0, 12), (-1, 0, 0), '2f85', 'cs66', 'plan-fails'),
    ('ring_inside_only_2f85', ring(45, 28, 30), (28, 0, 15), (-1, 0, 0), '2f85', 'cs66', 'tray', {'modes': ['inside']}),
    ('ring_inside_only_hande', ring(45, 28, 30), (28, 0, 15), (-1, 0, 0), 'hande', 'cs66', 'tray', {'modes': ['inside']}),
    ('too_wide_cube', box(300, 300, 300), (150, 150, 300), (0, 0, 1), '2f85', 'cs66', 'plan-fails'),
]


def run_case(case, python, workdir, snapshots):
    name, part, click, normal, gripper, robot, expected = case[:7]
    if callable(part):
        step = os.path.join(workdir, f'{name}.step')
        make_step(step, part)
    else:
        step = os.path.join(ROOT, part)
    plan = os.path.join(workdir, f'{name}.json')
    status = os.path.join(workdir, f'{name}_status.json')
    common = [python, os.path.join(BASE, 'auto_pick_place.py'), '--step', step, '--plan', plan, '--status', status,
              '--gripper', gripper, '--robot', robot]
    with open(os.path.join(BASE, 'sim_config.json')) as f:
        config = json.load(f)
    if len(case) > 7:  # extra grasp-planner settings for this case
        config['auto_grasp'] = case[7]
    # The camera stage adds about 40 s per run: only the cases that are about it use it.
    config['inspection']['enabled'] = 'camera' in name
    config_path = os.path.join(workdir, f'{name}_config.json')
    with open(config_path, 'w') as f:
        json.dump(config, f)
    common += ['--config', config_path]
    if len(click) == 6:  # two marked points
        where = ['--point1=' + ','.join(map(str, click)), '--point2=' + ','.join(map(str, normal))]
    else:
        where = ['--click=' + ','.join(map(str, click)), '--normal=' + ','.join(map(str, normal))]
    res = subprocess.run(common + ['--mode', 'plan'] + where, capture_output=True, text=True, cwd=BASE)
    with open(status) as f:
        st = json.load(f)
    if res.returncode != 0:
        return expected == 'plan-fails', f"plan failed: {st['message']}"
    if expected == 'plan-fails':
        return False, 'expected no plan, but one was found'
    with open(plan) as f:
        doc = json.load(f)
    best = doc['plans'][0]
    info = (f"{doc['seconds']:.0f}s plan, {best['mode']} {best['widthMm']:.0f}mm, "
            f"tilt {best['approachTiltDeg']:.0f}deg, {len(doc['plans'])} plans")
    extra = ['--headless', '--mode', 'execute']
    if snapshots:
        extra += ['--snapshots', os.path.join(snapshots, name)]
    res = subprocess.run(common + extra, capture_output=True, text=True, cwd=BASE)
    with open(status) as f:
        st = json.load(f)
    ok = res.returncode == 0 and st['info'].get('part_in_tray') is True
    if 'camera' in name:
        images = st['info'].get('inspection', [])
        missing = [im['file'] for im in images if not os.path.exists(os.path.join(workdir, im['file']))]
        info = f"{len(images)} camera images, skipped {len(best['inspectionSkipped'])} | {info}"
        if not images or missing:
            ok = False
            info = f'CAMERA IMAGES MISSING {missing} | {info}'
    errors = st['info'].get('finger_to_point_mm')
    if len(click) == 6 and (not errors or max(errors) > MAX_FINGER_ERROR_MM):
        ok = False
        info = f'FINGERS NOT ON THE POINTS {errors} | {info}'
    wanted_mode = expected.partition(':')[2]
    if wanted_mode and best['mode'] != wanted_mode:
        ok = False
        info = f'EXPECTED AN {wanted_mode.upper()} GRIP | {info}'
    return ok, f"{info} | {st['message']}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('filter', nargs='*')
    ap.add_argument('--snapshots')
    args = ap.parse_args()
    cases = [c for c in POINT_CASES + CASES if not args.filter or any(f in c[0] for f in args.filter)]
    failures = 0
    with tempfile.TemporaryDirectory() as workdir:
        for case in cases:
            ok, detail = run_case(case, sys.executable, workdir, args.snapshots)
            failures += not ok
            print(f"{'PASS' if ok else 'FAIL'}  {case[0]:24s} {detail}", flush=True)
    print(f'{len(cases) - failures}/{len(cases)} passed')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
