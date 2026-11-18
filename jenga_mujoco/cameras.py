"""
cameras.py - The default MuJoCo viewer framing and a fixed full-view camera of the cell (cam_full_view).
"""
import mujoco
import numpy as np

CAM_FULL_VIEW = 'cam_full_view'


def lookat_quat(eye, target, up=np.array([0.0, 0.0, 1.0])):
    """Computes a MuJoCo camera quaternion (w, x, y, z) looking from eye to target.
    
    MuJoCo cameras point along their local -Z axis with +Y as up and +X as right.
    """
    eye = np.asarray(eye, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    forward = target - eye
    norm_fwd = np.linalg.norm(forward)
    if norm_fwd < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    forward = forward / norm_fwd
    z_cam = -forward

    up = np.asarray(up, dtype=np.float64)
    right = np.cross(forward, up)
    if np.linalg.norm(right) < 1e-6:
        up_alt = np.array([0.0, 1.0, 0.0]) if abs(up[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
        right = np.cross(forward, up_alt)
    right = right / np.linalg.norm(right)
    y_cam = np.cross(z_cam, right)
    y_cam = y_cam / np.linalg.norm(y_cam)

    R = np.column_stack([right, y_cam, z_cam])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, R.flatten())
    return quat


def add_cameras(spec):
    """Default viewer framing (spec.stat / visual.global_) and a fixed camera showing the whole cell."""
    spec.stat.center = [0.35, 0.0, 0.2]
    spec.stat.extent = 1.3
    spec.visual.global_.azimuth = 145.0
    spec.visual.global_.elevation = -25.0
    eye, target = np.array([1.3, -1.1, 0.95]), np.array([0.3, 0.0, 0.2])
    body = spec.worldbody.add_body(name='cam_full_view_body', pos=eye, quat=lookat_quat(eye, target))
    body.add_camera(name=CAM_FULL_VIEW).fovy = 55.0
