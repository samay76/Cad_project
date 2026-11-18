import os
import argparse
import mujoco, mujoco.viewer
from world import add_world
from table_base import MOUNT_PLATE_TOP_Z
from cameras import add_cameras
from frames import T_gbase_tcp, T_to_pos_quat
from robots import get_robot, ROBOTS, DEFAULT_ROBOT

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def build(num_blocks=1, obj='jenga', cad_meta=None, part_pos=(0.45, -0.10, 0.03), part_quat=(1, 0, 0, 0),
          tcp_offset_mm=145.0, gravcomp=False, fine_fingers=False, gripper='2f85', return_spec=False, extra=None,
          include_sensor_cameras=True, robot=DEFAULT_ROBOT, tray_half=None, pick_station=None, coupling_mm=0.0,
          base_offset_mm=(0.0, 0.0, 0.0)):
    """Builds the arm + gripper scene (default: CS66 + Robotiq 2F-85).

    obj='jenga'    : original Jenga blocks (default; teleop / recorder / eval unchanged)
    obj='cad_part' : a converted STEP part (cad_meta from step_to_mesh.convert) on a free joint
    gravcomp       : compensate arm/gripper weight (an ideal cobot controller); off for the Jenga
                     scene so recorded/trained dynamics stay as they were
    fine_fingers   : decompose the finger meshes into several convex pieces (finger_collision.py) so
                     thin fingertips fit where they really fit (inside grips); off for Jenga
    gripper        : '2f85' (Menagerie Robotiq 2F-85, default) or 'hande' (Robotiq Hand-E, hande.py)
    robot          : arm id from robots.py: 'cs66' (default) or 'ec66'
    tray_half      : inner half-size (x, y) of the target tray in m (CAD parts); None = the original tray
    pick_station   : (info, placement) from pick_station.from_config: the part is picked from it (CAD parts)
    extra(spec)    : optional callback to add more elements (e.g. the ghost gripper) before compiling
    base_offset_mm : calibration (x, y, z mm): where the real robot base really is relative to the table, compared
                     with the table CAD (e.g. a thicker mounting plate: z > 0). Moves the arm in the sim by that much,
                     so the planner sends the real arm to the real part. Measured with calibrate_robot.py.
    coupling_mm    : coupling plate between the tool flange and the gripper (the gripper sits this much further out
                     along the flange's +Z), so the sim gripper is where the real one is
    """
    rob = get_robot(robot)
    arm_xml = rob.xml_path
    grip_xml = os.path.join(BASE_DIR, 'robotiq_2f85', '2f85.xml')
    arm = mujoco.MjSpec.from_file(arm_xml)
    if gripper == 'hande':
        from hande import build_spec as build_hande
        grip = build_hande()
    elif gripper == '2f85':
        grip = mujoco.MjSpec.from_file(grip_xml)
    else:
        raise ValueError(f"Unknown gripper '{gripper}' (expected '2f85' or 'hande')")

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

    if fine_fingers and gripper == '2f85':
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

    if obj == 'jenga':
        add_world(arm, num_blocks=num_blocks)
    elif obj == 'cad_part':
        if cad_meta is None:
            raise ValueError("obj='cad_part' needs cad_meta (from step_to_mesh.convert)")
        from cad_part import add_cad_part
        add_world(arm, add_block=False, pick_station=pick_station,
                  **({'tray_half': tray_half} if tray_half is not None else {}))
        add_cad_part(arm, cad_meta, pos=part_pos, quat=part_quat)
    else:
        raise ValueError(f"Unknown object type '{obj}' (expected 'jenga' or 'cad_part')")

    add_cameras(arm, include_sensor_cameras=include_sensor_cameras, wrist_body=rob.flange_body,
                flange_pos=rob.flange_pos, flange_quat=rob.flange_quat)
    if extra is not None:
        extra(arm)
    if return_spec:
        return arm
    return arm.compile()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--object', choices=['jenga', 'cad_part'], default='jenga')
    ap.add_argument('--gripper', choices=['2f85', 'hande'], default='2f85')
    ap.add_argument('--robot', choices=list(ROBOTS), default=DEFAULT_ROBOT)
    ap.add_argument('--step', default=os.path.join(BASE_DIR, '..', 'tests', 'fixtures', 'skf_cam_roller_305703_c-2z_0.stp'))
    args = ap.parse_args()

    meta = None
    if args.object == 'cad_part':
        from step_to_mesh import convert
        meta = convert(args.step)
    model = build(obj=args.object, cad_meta=meta, gripper=args.gripper, robot=args.robot)
    print('bodies:', model.nbody, 'joints:', model.njnt, 'actuators:', model.nu, 'cameras:', model.ncam)
    if args.object == 'jenga' and args.robot == 'cs66' and args.gripper == '2f85':
        mujoco.mj_saveModel(model, 'cs66_2f85.mjb')
    mujoco.viewer.launch(model)
