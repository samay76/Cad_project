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


CAM_INSPECTION = 'cam_inspection'


def inspection_camera(cfg):
    """(camera position, look-at point, fovy) from config/cell.json "inspection", or None when it is off."""
    insp = (cfg or {}).get('inspection') or {}
    if not insp.get('enabled'):
        return None
    return (np.asarray(insp['camera_pos'], dtype=float), np.asarray(insp['look_at'], dtype=float),
            float(insp.get('fovy_deg', 45.0)))


def add_inspection_camera(spec, pos, look_at, fovy=45.0):
    """The inspection camera: a camera housing on a stand from the floor, aimed at look_at (where the robot shows
    the part), and a MuJoCo camera 'cam_inspection' that sees what it sees. The housing and the stand collide, so
    the planner keeps the arm and the part clear of them."""
    pos, look_at = np.asarray(pos, dtype=float), np.asarray(look_at, dtype=float)
    G = mujoco.mjtGeom
    body = spec.worldbody.add_body(name='inspection_camera', pos=pos, quat=lookat_quat(pos, look_at))
    # camera frame: looks along -Z, +Y up
    body.add_geom(name='inspection_cam_housing', type=G.mjGEOM_BOX, size=[0.045, 0.016, 0.022], pos=[0, 0, 0.022],
                  rgba=[0.12, 0.13, 0.15, 1])
    body.add_geom(name='inspection_cam_lens', type=G.mjGEOM_CYLINDER, size=[0.011, 0.004], pos=[0, 0, -0.003],
                  rgba=[0.05, 0.25, 0.45, 1], contype=0, conaffinity=0)
    body.add_camera(name=CAM_INSPECTION, fovy=fovy)
    # stand: from the floor (table feet, ~0.8 m below the table top) up to the housing
    floor, top = -0.80, pos[2] - 0.03
    spec.worldbody.add_geom(name='inspection_cam_stand', type=G.mjGEOM_CYLINDER, size=[0.012, (top - floor) / 2],
                            pos=[pos[0], pos[1], (top + floor) / 2], rgba=[0.55, 0.57, 0.6, 1])
