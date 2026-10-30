"""
frames.py - Every coordinate-frame transform between the web CAD tool and MuJoCo.

Notation: T_a_b = pose of frame b expressed in frame a (4x4, meters).

Web CAD tool (Three.js, Z-up, millimeters):
  part      : the STEP file's own frame (the viewer never recentres it)
  tcp       : gripper TCP, between the fingertips. +Z = approach, X = closing axis.
  urdf_base : robotiq_85_base_link of public/gripper/robotiq_2f_85.urdf
              T_urdfbase_tcp = Trans(0, 0, tcpOffset)            (GripperModel.updateTcpOffset)
  Exported grasp: SavedGrasp.tcpPose = T_part_tcp, position in mm,
                  orientation quaternion in (x, y, z, w) order.

MuJoCo (Z-up, meters, quaternions in (w, x, y, z) order):
  part      : body 'cad_part' == STEP frame scaled by 0.001 (step_to_mesh.py)
  g_base    : gripper 'base' body (attached to the CS66 flange): Menagerie Robotiq 2F-85, or the
              Robotiq Hand-E built in hande.py (whose base frame IS the URDF root, no offset)
  g_tcp     : site on g_base at T_gbase_tcp = T_gbase_urdfbase * T_urdfbase_tcp

T_gbase_urdfbase - derived by matching the two gripper models' joint origins:
  URDF  left_knuckle (0.0306, 0, 0.0549)  inner_knuckle (0.0127, 0, 0.0614)
  MJCF  left_driver (-0.0306, ., 0.0657)  spring_link (-0.0127, ., 0.0722)
  => translation (0, 0, 0.0108) (Menagerie's base-coupling plate) and a 180 deg
     rotation about Z (left/right are swapped). The gripper is symmetric about
     its approach axis, so this only matters for which finger is called "left".

Chain used by the simulator:
  T_world_tcp_target = T_world_part(sim, after settling) * T_part_tcp(JSON)
  T_world_gbase      = T_world_tcp_target * inv(T_gbase_tcp)     (ghost gripper)
  IK drives site g_tcp to T_world_tcp_target.
"""
import numpy as np
import mujoco

MM = 0.001

# Menagerie 2F-85 base -> URDF robotiq_85_base_link
GBASE_TO_URDFBASE_POS = np.array([0.0, 0.0, 0.0108])
GBASE_TO_URDFBASE_QUAT = np.array([0.0, 0.0, 0.0, 1.0])  # wxyz, 180 deg about Z


def quat_xyzw_to_wxyz(q):
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


def make_T(pos, quat_wxyz):
    q = np.asarray(quat_wxyz, dtype=float)
    q = q / np.linalg.norm(q)
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    T = np.eye(4)
    T[:3, :3] = R.reshape(3, 3)
    T[:3, 3] = pos
    return T


def T_to_pos_quat(T):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(T[:3, :3]).reshape(9))
    return T[:3, 3].copy(), q


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def rot_angle_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def web_pose_to_T(pose):
    """Web `Pose` {position{x,y,z} mm, orientation{x,y,z,w}} -> 4x4 in meters."""
    p, o = pose['position'], pose['orientation']
    return make_T(np.array([p['x'], p['y'], p['z']]) * MM, quat_xyzw_to_wxyz([o['x'], o['y'], o['z'], o['w']]))


def T_gbase_tcp(tcp_offset_mm, gripper='2f85'):
    """Pose of the web TCP on the gripper base body 'g_base'.
    2F-85: Menagerie base differs from the URDF base (see above). Hand-E (hande.py): base == URDF root."""
    if gripper == 'hande':
        return make_T([0.0, 0.0, tcp_offset_mm * MM], [1, 0, 0, 0])
    T_gb_ub = make_T(GBASE_TO_URDFBASE_POS, GBASE_TO_URDFBASE_QUAT)
    T_ub_tcp = make_T([0.0, 0.0, tcp_offset_mm * MM], [1, 0, 0, 0])
    return T_gb_ub @ T_ub_tcp


def body_T(model, data, body_name):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    T = np.eye(4)
    T[:3, :3] = data.xmat[bid].reshape(3, 3)
    T[:3, 3] = data.xpos[bid]
    return T


def site_T(model, data, site_name):
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    T = np.eye(4)
    T[:3, :3] = data.site_xmat[sid].reshape(3, 3)
    T[:3, 3] = data.site_xpos[sid]
    return T


def pose_error(T_a, T_b):
    """(position error mm, rotation error deg)."""
    return float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]) / MM), rot_angle_deg(T_a[:3, :3], T_b[:3, :3])
