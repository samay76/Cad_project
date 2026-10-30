import os
import argparse
import mujoco, mujoco.viewer
from world import add_world
from cameras import add_cameras
from frames import T_gbase_tcp, T_to_pos_quat

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def build(num_blocks=1, obj='jenga', cad_meta=None, part_pos=(0.45, -0.10, 0.03), part_quat=(1, 0, 0, 0),
          tcp_offset_mm=145.0, gravcomp=False, fine_fingers=False, gripper='2f85', return_spec=False, extra=None):
    """Builds the CS66 + Robotiq 2F-85 scene.

    obj='jenga'    : original Jenga blocks (default; teleop / recorder / eval unchanged)
    obj='cad_part' : a converted STEP part (cad_meta from step_to_mesh.convert) on a free joint
    gravcomp       : compensate arm/gripper weight (an ideal cobot controller); off for the Jenga
                     scene so recorded/trained dynamics stay as they were
    fine_fingers   : decompose the finger meshes into several convex pieces (finger_collision.py) so
                     thin fingertips fit where they really fit (inside grips); off for Jenga
    gripper        : '2f85' (Menagerie Robotiq 2F-85, default) or 'hande' (Robotiq Hand-E, hande.py)
    extra(spec)    : optional callback to add more elements (e.g. the ghost gripper) before compiling
    """
    arm_xml = os.path.join(BASE_DIR, 'cs66', 'cs66.xml')
    grip_xml = os.path.join(BASE_DIR, 'robotiq_2f85', '2f85.xml')
    arm = mujoco.MjSpec.from_file(arm_xml)
    if gripper == 'hande':
        from hande import build_spec as build_hande
        grip = build_hande()
    elif gripper == '2f85':
        grip = mujoco.MjSpec.from_file(grip_xml)
    else:
        raise ValueError(f"Unknown gripper '{gripper}' (expected '2f85' or 'hande')")

    KP = [2000, 2000, 1500, 400, 400, 200]   # starting guesses
    KV = [200, 200, 150, 40, 40, 20]
    for j, kp, kv in zip(list(arm.joints)[:6], KP, KV):
        j.damping = [kv, 0, 0]
        a = arm.add_actuator(name='act_' + j.name, target=j.name,
                             trntype=mujoco.mjtTrn.mjTRN_JOINT,
                             gaintype=mujoco.mjtGain.mjGAIN_FIXED,
                             biastype=mujoco.mjtBias.mjBIAS_AFFINE)
        a.gainprm[0] = kp
        a.biasprm[:3] = [0, -kp, -kv]
        a.ctrlrange = [j.range[0], j.range[1]]
        a.forcerange = [-150, 150]

    arm.option.impratio = 10
    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC

    # Web-tool TCP on the gripper (see frames.py). Becomes 'g_tcp' after attach.
    tcp_pos, tcp_quat = T_to_pos_quat(T_gbase_tcp(tcp_offset_mm, gripper))
    grip.body('base').add_site(name='tcp', pos=tcp_pos, quat=tcp_quat, size=[0.003, 0.003, 0.003],
                               rgba=[0, 1, 1, 0.8], group=4)

    if fine_fingers and gripper == '2f85':
        from finger_collision import refine
        refine(grip)

    site = arm.body('wrist_3_link').add_site(
        name='gripper_mount', pos=[0, 0, 0], quat=[1, 0, 0, 0])
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
        add_world(arm, add_block=False)
        add_cad_part(arm, cad_meta, pos=part_pos, quat=part_quat)
    else:
        raise ValueError(f"Unknown object type '{obj}' (expected 'jenga' or 'cad_part')")

    add_cameras(arm)
    if extra is not None:
        extra(arm)
    if return_spec:
        return arm
    return arm.compile()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--object', choices=['jenga', 'cad_part'], default='jenga')
    ap.add_argument('--gripper', choices=['2f85', 'hande'], default='2f85')
    ap.add_argument('--step', default=os.path.join(BASE_DIR, '..', 'skf_cam_roller_305703_c-2z_0.stp'))
    args = ap.parse_args()

    meta = None
    if args.object == 'cad_part':
        from step_to_mesh import convert
        meta = convert(args.step)
    model = build(obj=args.object, cad_meta=meta, gripper=args.gripper)
    print('bodies:', model.nbody, 'joints:', model.njnt, 'actuators:', model.nu, 'cameras:', model.ncam)
    if args.object == 'jenga':
        mujoco.mj_saveModel(model, 'cs66_2f85.mjb')
    mujoco.viewer.launch(model)
