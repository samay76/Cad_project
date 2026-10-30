import mujoco
import numpy as np

# Camera name identifiers
CAM_FULL_VIEW = 'cam_full_view'
CAM_WRIST = 'cam_wrist_d405'
CAM_OVERVIEW = 'cam_overview_d405'
CAM_LOW_ANGLE = 'cam_low_angle_d435i'

CAMERA_NAMES = [CAM_FULL_VIEW, CAM_WRIST, CAM_OVERVIEW, CAM_LOW_ANGLE]

G = mujoco.mjtGeom


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
    """Configures default viewing parameters and adds all cameras to the spec:

    0. Default Viewer Free Camera & Full View (cam_full_view):
       - Configures spec.stat (center & extent) and spec.visual.global_
         so the default viewer opens with a clean, centered full view.
       - Adds a dedicated fixed full view camera (cam_full_view) showing the
         entire table, robot arm, workspace, and stands.

    1. Camera 1 (cam_wrist_d405):
       - Model: Intel RealSense D405
       - Mounting: On wrist_3_link
       - View: Close-up, ego-centric view of grasp and placement
       - FOV: 58° vertical (approx 87° x 58° RGB/depth)

    2. Camera 2 (cam_overview_d405):
       - Model: Intel RealSense D405
       - Mounting: Fixed in world (high angled overview)
       - View: Table, taped boundary, block stacks, and robot workspace
       - FOV: 58° vertical

    3. Camera 3 (cam_low_angle_d435i):
       - Model: Intel RealSense D435i
       - Mounting: Fixed in world (low viewpoint at table level)
       - View: Low-angle side view for tower height and block alignment
       - FOV: 42° vertical (approx 69° x 42° RGB)
    """
    # -------------------------------------------------------------------------
    # 0. Default Free Camera Setup & Full View Camera (cam_full_view)
    # -------------------------------------------------------------------------
    # Set default center & extent so viewer opens with a centered full view
    spec.stat.center = [0.35, 0.0, 0.2]
    spec.stat.extent = 1.3
    spec.visual.global_.azimuth = 145.0
    spec.visual.global_.elevation = -25.0

    eye_full = np.array([1.3, -1.1, 0.95])
    tgt_full = np.array([0.3, 0.0, 0.2])
    q_full = lookat_quat(eye_full, tgt_full, up=[0.0, 0.0, 1.0])

    cam_full_body = spec.worldbody.add_body(name='cam_full_view_body', pos=eye_full, quat=q_full)
    cam_full = cam_full_body.add_camera(name=CAM_FULL_VIEW)
    cam_full.fovy = 55.0

    # -------------------------------------------------------------------------
    # 1. Camera 1 — Wrist Intel RealSense D405
    # -------------------------------------------------------------------------
    wrist = spec.body('wrist_3_link')
    eye1 = np.array([0.0, -0.07, 0.04])       # Mounted offset on wrist flange
    tgt1 = np.array([0.0, 0.0, 0.16])        # Grasp center between finger pads
    q1 = lookat_quat(eye1, tgt1, up=[0.0, 0.0, 1.0])

    cam1_body = wrist.add_body(name='cam_wrist_body', pos=eye1, quat=q1)
    cam1 = cam1_body.add_camera(name=CAM_WRIST)
    cam1.fovy = 58.0

    # D405 compact casing: 42 x 42 x 23 mm (placed behind optical center in +z)
    cam1_body.add_geom(name='cam_wrist_housing', type=G.mjGEOM_BOX,
                       size=[0.021, 0.021, 0.0115], pos=[0.0, 0.0, 0.012],
                       rgba=[0.18, 0.18, 0.18, 1.0], contype=0, conaffinity=0)
    cam1_body.add_geom(name='cam_wrist_lens', type=G.mjGEOM_CYLINDER,
                       size=[0.007, 0.001], pos=[0.0, 0.0, 0.001],
                       rgba=[0.1, 0.25, 0.45, 1.0], contype=0, conaffinity=0)
    # Mounting bracket to wrist flange
    cam1_body.add_geom(name='cam_wrist_bracket', type=G.mjGEOM_BOX,
                       size=[0.012, 0.018, 0.003], pos=[0.0, 0.028, 0.014],
                       rgba=[0.3, 0.3, 0.3, 1.0], contype=0, conaffinity=0)

    # -------------------------------------------------------------------------
    # 2. Camera 2 — Overview Intel RealSense D405
    # -------------------------------------------------------------------------
    eye2 = np.array([0.35, -1.05, 0.95])      # High angled vantage in front of table
    tgt2 = np.array([0.35, 0.05, 0.15])       # Workspace & table center
    q2 = lookat_quat(eye2, tgt2, up=[0.0, 0.0, 1.0])

    cam2_body = spec.worldbody.add_body(name='cam_overview_body', pos=eye2, quat=q2)
    cam2 = cam2_body.add_camera(name=CAM_OVERVIEW)
    cam2.fovy = 58.0

    cam2_body.add_geom(name='cam_overview_housing', type=G.mjGEOM_BOX,
                       size=[0.021, 0.021, 0.0115], pos=[0.0, 0.0, 0.012],
                       rgba=[0.18, 0.18, 0.18, 1.0], contype=0, conaffinity=0)
    cam2_body.add_geom(name='cam_overview_lens', type=G.mjGEOM_CYLINDER,
                       size=[0.007, 0.001], pos=[0.0, 0.0, 0.001],
                       rgba=[0.1, 0.25, 0.45, 1.0], contype=0, conaffinity=0)
    # Stand pole from floor up to overview camera
    spec.worldbody.add_geom(name='cam_overview_stand', type=G.mjGEOM_CYLINDER,
                           size=[0.015, 0.85], pos=[0.35, -1.05, 0.10],
                           rgba=[0.25, 0.25, 0.25, 1.0], contype=0, conaffinity=0)

    # -------------------------------------------------------------------------
    # 3. Camera 3 — Low-Angle Intel RealSense D435i
    # -------------------------------------------------------------------------
    eye3 = np.array([0.95, -0.25, 0.16])      # Side view near table surface level
    tgt3 = np.array([0.55, 0.15, 0.12])       # Stack / boundary center
    q3 = lookat_quat(eye3, tgt3, up=[0.0, 0.0, 1.0])

    cam3_body = spec.worldbody.add_body(name='cam_low_angle_body', pos=eye3, quat=q3)
    cam3 = cam3_body.add_camera(name=CAM_LOW_ANGLE)
    cam3.fovy = 42.0

    # D435i horizontal form factor: 90 x 25 x 25 mm
    cam3_body.add_geom(name='cam_low_angle_housing', type=G.mjGEOM_BOX,
                       size=[0.045, 0.0125, 0.0125], pos=[0.0, 0.0, 0.013],
                       rgba=[0.75, 0.75, 0.78, 1.0], contype=0, conaffinity=0)
    cam3_body.add_geom(name='cam_low_angle_glass', type=G.mjGEOM_BOX,
                       size=[0.042, 0.010, 0.001], pos=[0.0, 0.0, 0.001],
                       rgba=[0.1, 0.1, 0.1, 1.0], contype=0, conaffinity=0)
    # Low table-edge mount stand
    spec.worldbody.add_geom(name='cam_low_angle_stand', type=G.mjGEOM_CYLINDER,
                           size=[0.012, 0.08], pos=[0.95, -0.25, 0.08],
                           rgba=[0.25, 0.25, 0.25, 1.0], contype=0, conaffinity=0)


_RENDERER_CACHE = {}


def get_cached_renderer(model, width=640, height=480):
    """Retrieves or creates a persistent renderer instance, avoiding expensive context recreation."""
    key = (id(model), int(width), int(height))
    if key not in _RENDERER_CACHE:
        _RENDERER_CACHE[key] = mujoco.Renderer(model, int(height), int(width))
    return _RENDERER_CACHE[key]


def render_camera(model, data, camera_name=None, width=640, height=480):
    """Renders a single frame from the specified camera as a uint8 RGB numpy array using cached renderer."""
    renderer = get_cached_renderer(model, width, height)
    if camera_name is not None:
        renderer.update_scene(data, camera=camera_name)
    else:
        renderer.update_scene(data)
    return renderer.render()


def render_all_cameras(model, data, width=640, height=480):
    """Renders a single frame from all cameras and returns a dictionary of RGB images."""
    images = {}
    renderer = get_cached_renderer(model, width, height)
    for name in CAMERA_NAMES:
        renderer.update_scene(data, camera=name)
        images[name] = renderer.render()
    # Also include default free camera view
    renderer.update_scene(data)
    images['free_camera_default'] = renderer.render()
    return images
