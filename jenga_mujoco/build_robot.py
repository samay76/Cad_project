import os
import argparse
import mujoco, mujoco.viewer
from world import add_world
from table_base import MOUNT_PLATE_TOP_Z
from cameras import add_cameras
from frames import T_gbase_tcp, T_to_pos_quat
from robots import get_robot, DEFAULT_ROBOT

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def build(cad_meta=None, part_pos=(0.45, -0.10, 0.03), part_quat=(1, 0, 0, 0), tcp_offset_mm=145.0, gravcomp=False,
          fine_fingers=False, return_spec=False, extra=None, robot=DEFAULT_ROBOT, tray_xy=None, tray_half=None,
          pick_station=None, coupling_mm=0.0, base_offset_mm=(0.0, 0.0, 0.0), gripper='2f85'):
    """Builds the cell: the EC66 arm with the Robotiq 2F-85, the table, tray and pick fixture, and the part.

    cad_meta       : a converted STEP part (step_to_mesh.convert) on a free joint at part_pos/part_quat; None = no part
    gravcomp       : compensate arm/gripper weight (an ideal cobot controller)
    fine_fingers   : decompose the finger meshes into several convex pieces (finger_collision.py) so thin
                     fingertips fit where they really fit
    tray_xy        : centre of the target tray (sim_config.json place_xy); tray_half its inner half-size (m)
    pick_station   : (info, placement, seat_part) from pick_station.from_config: the fixture the part is picked from
    extra(spec)    : optional callback to add more elements (e.g. the ghost gripper) before compiling
    base_offset_mm : calibration (x, y, z mm): where the real robot base really is relative to the table CAD.
                     Measured with calibrate_robot.py.
    coupling_mm    : coupling plate between the tool flange and the gripper (the gripper sits this much further out
                     along the flange's +Z), so the sim gripper is where the real one is
    """
    if gripper != '2f85':
        raise ValueError(f"Unknown gripper '{gripper}' (only the Robotiq 2F-85 '2f85')")
    rob = get_robot(robot)
    arm_xml = rob.xml_path
    grip_xml = os.path.join(BASE_DIR, 'robotiq_2f85', '2f85.xml')
    arm = mujoco.MjSpec.from_file(arm_xml)
    grip = mujoco.MjSpec.from_file(grip_xml)

    for j, kp, kv in zip(list(arm.joints)[:6], rob.kp, rob.kv):
        j.damping = [kv, 0, 0]
        a = arm.add_actuator(name='act_' + j.name, target=j.name,
                             trntype=mujoco.mjtTrn.mjTRN_JOINT,
                             gaintype=mujoco.mjtGain.mjGAIN_FIXED,
                             biastype=mujoco.mjtBias.mjBIAS_AFFINE)
        a.gainprm[0] = kp
        a.biasprm[:3] = [0, -kp, -kv]
        a.ctrlrange = [j.range[0], j.range[1]]
        a.forcerange = [-150, 150]

    # The arm stands on the table's mounting plate (Table_base.step), 15 mm above the table top.
    off = [float(v) / 1000.0 for v in base_offset_mm]
    arm.body(rob.base_bodies[0]).pos = [off[0], off[1], MOUNT_PLATE_TOP_Z + off[2]]

    arm.option.impratio = 10
    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC

    # Web-tool TCP on the gripper (see frames.py). Becomes 'g_tcp' after attach.
    tcp_pos, tcp_quat = T_to_pos_quat(T_gbase_tcp(tcp_offset_mm, gripper))
    grip.body('base').add_site(name='tcp', pos=tcp_pos, quat=tcp_quat, size=[0.003, 0.003, 0.003],
                               rgba=[0, 1, 1, 0.8], group=4)

    if fine_fingers:
        from finger_collision import refine
        refine(grip)

    mount_pos = list(rob.flange_pos)
    if coupling_mm:
        import numpy as np
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, np.asarray(rob.flange_quat, dtype=float))
        axis = R.reshape(3, 3)[:, 2]
        mount_pos = (np.asarray(rob.flange_pos) + axis * coupling_mm / 1000.0).tolist()
        # The coupling plate itself (75 mm diameter, like Robotiq's), between the flange and the gripper. It
        # collides, so the planner keeps it clear of the table and the part too.
        arm.body(rob.flange_body).add_geom(
            name='coupling', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.0375, coupling_mm / 2000.0, 0],
            pos=(np.asarray(rob.flange_pos) + axis * coupling_mm / 2000.0).tolist(), quat=list(rob.flange_quat),
            rgba=[0.2, 0.2, 0.22, 1])
    site = arm.body(rob.flange_body).add_site(
        name='gripper_mount', pos=mount_pos, quat=list(rob.flange_quat))
    arm.attach(grip, site=site, prefix='g_')
    if gravcomp:
        for b in arm.bodies:
            if b.name != 'world':
                b.gravcomp = 1.0

    world_kw = {k: v for k, v in (('tray_xy', tray_xy), ('tray_half', tray_half)) if v is not None}
    add_world(arm, pick_station=pick_station, **world_kw)
    if cad_meta is not None:
        from cad_part import add_cad_part
        add_cad_part(arm, cad_meta, pos=part_pos, quat=part_quat)

    add_cameras(arm)
    if extra is not None:
        extra(arm)
    if return_spec:
        return arm
    return arm.compile()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Open the cell in the MuJoCo viewer (optionally with a STEP part).')
    ap.add_argument('--step', help='STEP/STL part to show on the table')
    args = ap.parse_args()
    meta = None
    if args.step:
        from step_to_mesh import convert
        meta = convert(args.step)
    model = build(cad_meta=meta)
    print('bodies:', model.nbody, 'joints:', model.njnt, 'actuators:', model.nu, 'cameras:', model.ncam)
    mujoco.viewer.launch(model)
