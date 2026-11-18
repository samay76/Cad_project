"""
live_object_pointcloud.py
=========================

ONE window, LIVE: the camera frame with the object's 3D point cloud painted on the object,
plus a rotatable 3D view of the object next to it. Updates every frame.

How the object is found (every frame, no set-up step needed)
------------------------------------------------------------
1. Support plane : the table / plate the object stands on is found with RANSAC
                   (re-fitted automatically every few seconds, or with 't').
                   If several big planes are visible, the FARTHEST one is the support.
2. Above table   : every pixel more than HEIGHT_MIN above that plane is a candidate.
3. Depth edges   : candidates are cut wherever the depth jumps (object vs. things behind it).
4. Pick          : the connected region closest to the image center = the object.
5. Clean         : 3D points -> voxel grid -> statistical outlier removal.
If no plane is visible (object held in the air), everything inside MIN..MAX depth is used.

The table is tinted BLUE in the image so you can see what the script thinks is the table.

Camera distance: the D435i cannot measure closer than ~0.2-0.3 m. Keep the object about
0.3-0.6 m from the camera; in the depth view ('d') the object must NOT be black.

Keys (click the window first)
-----------------------------
    q / ESC   quit
    m         build the CAD model: fuses MODEL_FRAMES frames (keep camera + part still),
              fills the measured top surface down to the table -> gray solid mesh,
              saved automatically as output/model_<time>.stl (millimeters)
    p         right panel: CAD model <-> live point cloud
    v         open the model in a full Open3D viewer window (close it to continue)
    s         save the current object cloud -> output/object_<time>.ply
    f         freeze / unfreeze
    t         re-fit the table plane now
    c         point colors: depth colormap <-> real RGB
    d         show the depth image instead of the 3D view on the right
    r         auto-rotate the 3D view (drag with the mouse to rotate)
    + / -     MAX_DEPTH +/- 5 cm
"""

import os
import sys
import time
import warnings
from datetime import datetime

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

# =============================================================================
# Settings
# =============================================================================

STREAM_MODES = [(848, 480, 30), (640, 480, 30), (640, 480, 15), (424, 240, 15)]
WARMUP_FRAMES = 30

MIN_DEPTH = 0.15              # m
MAX_DEPTH = 0.80              # m  - set just behind the table/object to cut the room away

# Support plane (table)
PLANE_DISTANCE = 0.005        # RANSAC inlier distance (m)
PLANE_MIN_FRACTION = 0.15     # a plane must hold >= 15 % of the points to count
PLANE_REFIT_SECONDS = 2.0     # re-fit automatically this often (handles a moved camera)
PLANE_SAMPLE_POINTS = 8000    # points used for the RANSAC fit (speed)

# Object segmentation
HEIGHT_MIN = 0.005            # m above the table to count as object (lower = more of the base)
HEIGHT_MAX = 0.50             # m above the table (ignore things far above, e.g. your arm)
EDGE_JUMP_ABS = 0.006         # depth jump (m) that separates two surfaces ...
EDGE_JUMP_REL = 0.03          # ... plus 3 % of the distance
MIN_OBJECT_AREA_PX = 300      # smaller regions are ignored
MASK_CLOSE_PX = 7             # close gaps up to this size (holes, dark spots without depth)
CENTER_WEIGHT_RADIUS = 0.35   # objects farther than ~35 % of the image diagonal from the
                              # center count much less (bigger = center matters less)
TRACKING_BONUS = 2.0          # prefer the region that overlaps last frame's object (no jumping)

# Point cloud
VOXEL_SIZE = 0.002            # 2 mm
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 2.0

# CAD model ('m'): camera + part fixed. The measured top surface (height above the table)
# is filled straight down to the table -> solid, watertight mesh -> STL in mm.
MODEL_FRAMES = 30             # frames fused for the model (median per cell = low noise)
MODEL_CELL_MM = 1.0           # grid resolution of the model (mm)
MODEL_MIN_VALID = 0.3         # a cell must be measured in >= 30 % of the frames
MODEL_MARGIN_M = 0.01         # grid margin around the object (m)
MODEL_SMOOTH_STEP_MM = 1.5    # smoothing strength; 0 = off (raw, rippled surface)
MODEL_COLOR = (0.72, 0.72, 0.75)   # plain gray

# Display
DOT_SIZE = 2
BACKGROUND_DIM = 0.4
WINDOW_NAME = "Live object point cloud  (s save | f freeze | t table | c colors | d depth | q quit)"
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


# =============================================================================
# Camera
# =============================================================================

def start_camera():
    """Find the camera and start the best stream mode the USB link supports."""
    devices = rs.context().query_devices()
    if len(devices) == 0:
        sys.exit("ERROR: no RealSense camera found. Check the cable / close realsense-viewer.")
    dev = devices[0]
    usb = (dev.get_info(rs.camera_info.usb_type_descriptor)
           if dev.supports(rs.camera_info.usb_type_descriptor) else "?")
    print(f"Camera: {dev.get_info(rs.camera_info.name)}  USB {usb}")
    if usb.startswith("2"):
        print("WARNING: camera is on USB 2 -> lower resolution / FPS. "
              "Plug it directly into a USB 3 port for 848x480 @ 30 fps.")
    pipeline = rs.pipeline()
    for w, h, fps in STREAM_MODES:
        cfg = rs.config()
        cfg.enable_device(dev.get_info(rs.camera_info.serial_number))
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        try:
            profile = pipeline.start(cfg)
            print(f"Streaming {w}x{h} @ {fps} fps")
            return pipeline, profile
        except RuntimeError:
            continue
    sys.exit("ERROR: the camera refused every stream mode.")


def make_filters():
    """depth->disparity, spatial, temporal, disparity->depth.
    (No hole filling: it invents depth at object edges, which hurts segmentation.)"""
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)
    temporal = rs.temporal_filter()
    temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
    temporal.set_option(rs.option.filter_smooth_delta, 20)
    return [rs.disparity_transform(True), spatial, temporal, rs.disparity_transform(False)]


# =============================================================================
# Geometry
# =============================================================================

_rays = {}


def deproject(depth_m, intr):
    """Depth image -> (H, W, 3) array of 3D points in camera coords (meters)."""
    key = (intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
    if key not in _rays:
        u, v = np.meshgrid(np.arange(intr.width, dtype=np.float32),
                           np.arange(intr.height, dtype=np.float32))
        _rays.clear()
        _rays[key] = ((u - intr.ppx) / intr.fx, (v - intr.ppy) / intr.fy)
    rx, ry = _rays[key]
    return np.dstack((rx * depth_m, ry * depth_m, depth_m))


def fit_support_plane(P, usable):
    """RANSAC: find up to 3 big planes, return the farthest one as (normal, d) with the
    normal pointing towards the camera, so height = P @ n + d is > 0 above the table."""
    pts = P[usable]
    if len(pts) < 500:
        return None
    if len(pts) > PLANE_SAMPLE_POINTS:
        pts = pts[np.random.choice(len(pts), PLANE_SAMPLE_POINTS, replace=False)]
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts.astype(np.float64)))
    total, planes = len(pts), []
    for _ in range(3):
        if len(pcd.points) < 300:
            break
        (a, b, c, d), inliers = pcd.segment_plane(PLANE_DISTANCE, 3, 300)
        if len(inliers) < PLANE_MIN_FRACTION * total:
            break
        norm = np.linalg.norm([a, b, c])
        n, d = np.array([a, b, c]) / norm, d / norm
        if d < 0:                       # camera (origin) must be on the positive side
            n, d = -n, -d
        planes.append((d, n))           # d = distance camera -> plane
        pcd = pcd.select_by_index(inliers, invert=True)
    if not planes:
        return None
    d, n = max(planes, key=lambda p: p[0])
    return n.astype(np.float32), np.float32(d)


def depth_edges(z):
    """Pixels next to a depth jump (boundary between surfaces at different distances)."""
    thr = EDGE_JUMP_ABS + EDGE_JUMP_REL * z
    edge = np.zeros(z.shape, bool)
    jx = np.abs(np.diff(z, axis=1)) > thr[:, 1:]
    jy = np.abs(np.diff(z, axis=0)) > thr[1:, :]
    edge[:, 1:] |= jx
    edge[:, :-1] |= jx
    edge[1:, :] |= jy
    edge[:-1, :] |= jy
    return edge


def segment_object(depth_m, P, plane, max_depth, prev_mask=None):
    """Find the object. Returns (object_mask, cloud_mask, table_mask).

    object_mask : the whole object region (for display / tracking)
    cloud_mask  : object_mask minus depth-edge pixels ("flying pixels" that smear between
                  e.g. the top of a peg and the plate under it) -> used for the 3D points
    """
    valid = (depth_m > MIN_DEPTH) & (depth_m < max_depth)
    if plane is not None:
        n, d = plane
        height = P @ n + d
        table = valid & (np.abs(height) <= PLANE_DISTANCE * 1.5)
        cand = valid & (height > HEIGHT_MIN) & (height < HEIGHT_MAX)
    else:
        table = np.zeros_like(valid)
        cand = valid

    # Everything standing on the table that touches = one object (a peg on a plate stays
    # part of the plate). Small gaps (holes with no depth) are closed.
    m = cand.astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((MASK_CLOSE_PX, MASK_CLOSE_PX), np.uint8))
    n_lab, labels, stats, cents = cv2.connectedComponentsWithStats(m, connectivity=8)

    # Score = size x closeness to the image center x overlap with last frame's object.
    h, w = m.shape
    diag = np.hypot(w, h)
    best, best_score = 0, 0.0
    for i in range(1, n_lab):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < MIN_OBJECT_AREA_PX:
            continue
        dist = np.hypot(cents[i][0] - w / 2, cents[i][1] - h / 2) / (CENTER_WEIGHT_RADIUS * diag)
        score = area * np.exp(-dist ** 2)
        if prev_mask is not None and prev_mask.any():
            x, y, bw, bh = stats[i, :4]
            overlap = np.count_nonzero(prev_mask[y:y + bh, x:x + bw] & (labels[y:y + bh, x:x + bw] == i))
            score *= 1.0 + TRACKING_BONUS * overlap / area
        if score > best_score:
            best, best_score = i, score
    if best == 0:
        empty = np.zeros_like(m)
        return empty, empty, table

    obj = ((labels == best) * 255).astype(np.uint8)
    cloud = obj.copy()
    cloud[depth_edges(depth_m) | ~cand] = 0        # drop flying pixels and closed-gap pixels
    return obj, cloud, table


def mask_to_cloud(P, color_bgr, mask):
    sel = mask > 0
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P[sel].astype(np.float64)))
    pcd.colors = o3d.utility.Vector3dVector(color_bgr[sel][:, ::-1] / 255.0)
    if len(pcd.points) == 0:
        return pcd
    pcd = pcd.voxel_down_sample(VOXEL_SIZE)
    if len(pcd.points) > OUTLIER_NB_NEIGHBORS:
        pcd, _ = pcd.remove_statistical_outlier(OUTLIER_NB_NEIGHBORS, OUTLIER_STD_RATIO)
    return pcd


# =============================================================================
# CAD model: height map -> solid mesh
# =============================================================================

class HeightMapModel:
    """Collects the object's height above the table on a regular grid (in table
    coordinates, so it is independent of the camera's tilt) over several frames."""

    def __init__(self, plane, P, cloud_mask):
        n, d = plane
        x = np.array([1.0, 0.0, 0.0], np.float32)
        e1 = x - (x @ n) * n                       # in-plane axis along the camera's x
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)                       # (e1, e2, n) is right-handed, n = up
        self.axes, self.n, self.d = np.stack((e1, e2)), n, d
        self.cell = MODEL_CELL_MM / 1000.0
        uv = P[cloud_mask > 0] @ self.axes.T
        self.u0 = uv[:, 0].min() - MODEL_MARGIN_M
        self.v0 = uv[:, 1].max() + MODEL_MARGIN_M
        self.W = int(np.ceil((uv[:, 0].max() + MODEL_MARGIN_M - self.u0) / self.cell))
        self.H = int(np.ceil((self.v0 - uv[:, 1].min() + MODEL_MARGIN_M) / self.cell))
        self.frames = []

    def add(self, P, cloud_mask):
        pts = P[cloud_mask > 0]
        uv, h = pts @ self.axes.T, pts @ self.n + self.d
        c = ((uv[:, 0] - self.u0) / self.cell).astype(int)
        r = ((self.v0 - uv[:, 1]) / self.cell).astype(int)
        ok = (c >= 0) & (c < self.W) & (r >= 0) & (r < self.H) & (h > 0)
        idx = r[ok] * self.W + c[ok]
        s = np.bincount(idx, weights=h[ok], minlength=self.H * self.W)
        k = np.bincount(idx, minlength=self.H * self.W)
        grid = np.full(self.H * self.W, np.nan, np.float32)
        grid[k > 0] = s[k > 0] / k[k > 0]
        self.frames.append(grid.reshape(self.H, self.W))

    def build(self):
        """Fuse the frames and return a watertight gray TriangleMesh (meters, z = up,
        z = 0 on the table)."""
        stack = np.stack(self.frames)
        seen = np.mean(~np.isnan(stack), axis=0) >= MODEL_MIN_VALID
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            height = np.nanmedian(stack, axis=0)

        # Footprint = measured cells, small gaps closed, inner holes filled, largest blob.
        foot = cv2.morphologyEx(seen.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        flood = np.pad(foot, 1).copy()
        cv2.floodFill(flood, None, (0, 0), 2)
        foot = (flood[1:-1, 1:-1] != 2).astype(np.uint8)
        n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(foot, connectivity=4)
        if n_lab <= 1:
            return None
        foot = labels == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))

        # Cells touching only at a corner make the solid non-manifold -> add a bridge cell.
        while True:
            a, b, c, d = foot[:-1, :-1], foot[:-1, 1:], foot[1:, :-1], foot[1:, 1:]
            p1, p2 = a & d & ~b & ~c, b & c & ~a & ~d
            if not (p1.any() or p2.any()):
                break
            foot[:-1, 1:] |= p1
            foot[1:, :-1] |= p2

        # Fill unmeasured cells (shadows, dark spots) from their neighbors, then smooth.
        # (cv2.inpaint is done on uint16 in 0.01 mm units - it is unreliable on float32.)
        known = foot & seen & ~np.isnan(height)
        hm = np.round(np.where(known, height, 0) * 1e5).clip(0, 65535).astype(np.uint16)
        ring = cv2.dilate(foot.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        fill = (ring & ~known).astype(np.uint8)
        if fill.any():
            hm = cv2.inpaint(hm, fill, 3, cv2.INPAINT_TELEA)
        hm_mm = cv2.medianBlur(hm, 3).astype(np.float32) / 100.0
        # Edge-preserving smoothing: flattens the stereo "ripple" on flat faces but keeps
        # steps (holes, pegs, edges) higher than ~MODEL_SMOOTH_STEP_MM sharp.
        if MODEL_SMOOTH_STEP_MM > 0:
            hm_mm = cv2.bilateralFilter(hm_mm, 9, MODEL_SMOOTH_STEP_MM, 3)
        hm = hm_mm / 1000.0
        hm[~foot] = 0
        return heightmap_to_mesh(hm, foot, self.cell, self.u0, self.v0)


def heightmap_to_mesh(hm, foot, cell, x0, y0):
    """Extrude a height map into a closed solid: top surface + vertical walls + flat bottom."""
    H, W = hm.shape
    # Corner heights = mean of the footprint cells touching that corner.
    ph, pf = np.pad(hm * foot, 1), np.pad(foot.astype(np.float32), 1)
    sum_h = ph[:-1, :-1] + ph[:-1, 1:] + ph[1:, :-1] + ph[1:, 1:]
    sum_f = pf[:-1, :-1] + pf[:-1, 1:] + pf[1:, :-1] + pf[1:, 1:]
    used = sum_f > 0
    corner_h = np.where(used, sum_h / np.maximum(sum_f, 1), 0)

    ci, cj = np.nonzero(used)
    top_id = np.full(used.shape, -1, np.int64)
    top_id[ci, cj] = np.arange(len(ci))
    nv = len(ci)
    xy = np.column_stack((x0 + cj * cell, y0 - ci * cell))
    verts = np.vstack((np.column_stack((xy, corner_h[ci, cj])),      # top
                       np.column_stack((xy, np.zeros(nv)))))          # bottom (z = 0)

    r, c = np.nonzero(foot)
    TL, TR, BR, BL = top_id[r, c], top_id[r, c + 1], top_id[r + 1, c + 1], top_id[r + 1, c]
    tris, want = [], []                                               # triangle + outward dir
    up, down = np.array([0, 0, 1.0]), np.array([0, 0, -1.0])
    for t in (np.column_stack((TL, BL, BR)), np.column_stack((TL, BR, TR))):
        tris.append(t); want.append(np.tile(up, (len(t), 1)))
    for t in (np.column_stack((TL, BL, BR)), np.column_stack((TL, BR, TR))):
        tris.append(t + nv); want.append(np.tile(down, (len(t), 1)))

    pfoot = np.pad(foot, 1)
    sides = [  # (neighbor empty?, corner A, corner B, outward direction)
        (~pfoot[r + 1, c],     TL, BL, (-1, 0, 0)),   # left
        (~pfoot[r + 1, c + 2], TR, BR, (1, 0, 0)),    # right
        (~pfoot[r, c + 1],     TL, TR, (0, 1, 0)),    # row above  (= +y)
        (~pfoot[r + 2, c + 1], BL, BR, (0, -1, 0)),   # row below  (= -y)
    ]
    for empty, A, B, out in sides:
        a, b = A[empty], B[empty]
        for t in (np.column_stack((a, a + nv, b + nv)), np.column_stack((a, b + nv, b))):
            tris.append(t); want.append(np.tile(np.array(out, float), (len(t), 1)))

    tris, want = np.vstack(tris), np.vstack(want)
    v = verts[tris]
    normal = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    flip = np.einsum("ij,ij->i", normal, want) < 0                    # make all faces point out
    tris[flip] = tris[flip][:, ::-1]

    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts),
                                     o3d.utility.Vector3iVector(tris))
    mesh.remove_unreferenced_vertices()
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color(MODEL_COLOR)
    return mesh


def flat_shaded(mesh):
    """Copy with un-shared vertices so every triangle is lit flat (crisp CAD look)."""
    v = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].reshape(-1, 3)
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v),
                                  o3d.utility.Vector3iVector(np.arange(len(v)).reshape(-1, 3)))
    m.compute_vertex_normals()
    m.paint_uniform_color(MODEL_COLOR)
    return m


def export_stl(mesh):
    """Save the model as STL in millimeters (z = 0 is the table). Returns the path."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out = o3d.geometry.TriangleMesh(mesh)
    out.scale(1000.0, center=np.zeros(3))
    out.compute_triangle_normals()
    path = os.path.join(OUTPUT_DIR, f"model_{datetime.now():%Y%m%d_%H%M%S}.stl")
    o3d.io.write_triangle_mesh(path, out)
    return path


class ModelRenderer:
    """Renders the gray mesh into an image (hidden Open3D window) for the side panel."""

    def __init__(self, size):
        self.size = size
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(visible=False, width=size[0], height=size[1])
        self.vis.get_render_option().background_color = np.array([0.1, 0.1, 0.12])
        self.mesh = None

    def set_mesh(self, mesh):
        if self.mesh is not None:
            self.vis.remove_geometry(self.mesh, reset_bounding_box=False)
        self.mesh = flat_shaded(mesh)
        self.vis.add_geometry(self.mesh, reset_bounding_box=True)
        ctrl = self.vis.get_view_control()
        ctrl.set_front([0.4, -0.7, 0.6])     # look from above at an angle
        ctrl.set_up([0, 0, 1])
        ctrl.set_zoom(0.95)

    def render(self, dx=0.0, dy=0.0):
        if dx or dy:
            self.vis.get_view_control().rotate(dx, dy)
        self.vis.poll_events()
        self.vis.update_renderer()
        img = np.asarray(self.vis.capture_screen_float_buffer(do_render=True))
        return cv2.resize((img[:, :, ::-1] * 255).astype(np.uint8), self.size)

    def close(self):
        self.vis.destroy_window()


# =============================================================================
# Drawing
# =============================================================================

_JET = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1), cv2.COLORMAP_JET)[:, 0]


def point_colors(pts, rgb, mode):
    if mode == "rgb":
        return (rgb[:, ::-1] * 255).astype(np.uint8)
    z = pts[:, 2]
    t = (z - z.min()) / max(z.max() - z.min(), 1e-6)
    return _JET[(255 * (1 - t)).astype(np.uint8)]          # red = near, blue = far


def splat(img, u, v, colors, size):
    h, w = img.shape[:2]
    for du in range(size):
        for dv in range(size):
            uu, vv = u + du, v + dv
            ok = (uu >= 0) & (uu < w) & (vv >= 0) & (vv < h)
            img[vv[ok], uu[ok]] = colors[ok]


def draw_frame(color_bgr, pcd, mask, table, intr, mode):
    out = color_bgr.copy()
    if mask.any():
        dim = mask == 0
        out[dim] = (out[dim] * BACKGROUND_DIM).astype(np.uint8)
    out[table] = (0.6 * out[table] + 0.4 * np.array([255, 120, 0])).astype(np.uint8)  # table = blue
    pts = np.asarray(pcd.points)
    if pts.size:
        u = np.round(intr.fx * pts[:, 0] / pts[:, 2] + intr.ppx).astype(int)
        v = np.round(intr.fy * pts[:, 1] / pts[:, 2] + intr.ppy).astype(int)
        order = np.argsort(-pts[:, 2])
        splat(out, u[order], v[order], point_colors(pts, np.asarray(pcd.colors), mode)[order], DOT_SIZE)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours, -1, (0, 255, 0), 1)
        x, y, w, h = cv2.boundingRect(mask)
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 200, 255), 2)
    return out


def draw_depth(depth_m, max_depth, size):
    norm = np.clip((depth_m - MIN_DEPTH) / (max_depth - MIN_DEPTH), 0, 1)
    img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[depth_m <= 0] = 0
    img[depth_m > max_depth] = (60, 60, 60)
    return cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)


def draw_3d_panel(pcd, size, yaw, pitch, mode):
    panel = np.full((size[1], size[0], 3), 25, np.uint8)
    pts = np.asarray(pcd.points)
    if pts.size == 0:
        cv2.putText(panel, "no object", (size[0] // 2 - 55, size[1] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2, cv2.LINE_AA)
        return panel
    p = pts - pts.mean(axis=0)
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    rot = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    p = p @ rot.T
    scale = 0.42 * min(size) / max(np.linalg.norm(p, axis=1).max(), 1e-3)
    u = (p[:, 0] * scale + size[0] / 2).astype(int)
    v = (p[:, 1] * scale + size[1] / 2).astype(int)
    order = np.argsort(-p[:, 2])
    splat(panel, u[order], v[order], point_colors(pts, np.asarray(pcd.colors), mode)[order], DOT_SIZE + 1)
    return panel


def put_text(img, lines, x=10, y=22):
    """White text on a semi-transparent dark box (readable on any background)."""
    if not lines:
        return
    font, scale = cv2.FONT_HERSHEY_SIMPLEX, 0.55
    width = max(cv2.getTextSize(l, font, scale, 1)[0][0] for l in lines)
    x0, y0, x1, y1 = x - 6, y - 17, min(x + width + 6, img.shape[1]), y + 22 * (len(lines) - 1) + 8
    img[y0:y1, x0:x1] = (img[y0:y1, x0:x1] * 0.35).astype(np.uint8)
    for line in lines:
        cv2.putText(img, line, (x, y), font, scale, (255, 255, 255), 1, cv2.LINE_AA)
        y += 22


class View:
    """Mouse-drag rotation for the 3D panel."""
    def __init__(self, panel_x):
        self.panel_x, self.yaw, self.pitch = panel_x, 0.0, 0.0
        self.auto, self.drag = True, None
        self.dx = self.dy = 0.0           # pending drag (pixels) for the model renderer

    def on_mouse(self, event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN and x >= self.panel_x:
            self.drag, self.auto = (x, y), False
        elif event == cv2.EVENT_MOUSEMOVE and self.drag and flags & cv2.EVENT_FLAG_LBUTTON:
            self.yaw += (x - self.drag[0]) * 0.01
            self.pitch += (y - self.drag[1]) * 0.01
            self.dx += x - self.drag[0]
            self.dy += y - self.drag[1]
            self.drag = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag = None


# =============================================================================
# Main
# =============================================================================

def main():
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    pipeline, profile = start_camera()
    try:
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        filters = make_filters()
        for _ in range(WARMUP_FRAMES):
            pipeline.wait_for_frames()

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        view = None
        plane, plane_t = None, 0.0
        mask = None
        obj = o3d.geometry.PointCloud()
        max_depth, color_mode, frozen, show_depth = MAX_DEPTH, "depth", False, False
        fps, n_frames, t_fps = 0.0, 0, time.time()
        builder, model, renderer, show_model = None, None, None, False

        while True:
            frames = align.process(pipeline.wait_for_frames())
            depth_frame, color_frame = frames.get_depth_frame(), frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            for f in filters:
                depth_frame = f.process(depth_frame)
            depth_frame = depth_frame.as_depth_frame()
            intr = depth_frame.profile.as_video_stream_profile().get_intrinsics()
            depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
            color = np.asanyarray(color_frame.get_data())
            if view is None:
                view = View(panel_x=intr.width)
                cv2.setMouseCallback(WINDOW_NAME, view.on_mouse)

            P = deproject(depth_m, intr)
            valid = (depth_m > MIN_DEPTH) & (depth_m < max_depth)

            # (Re-)fit the table plane periodically, ignoring the current object.
            if time.time() - plane_t > PLANE_REFIT_SECONDS:
                usable = valid if mask is None else valid & (mask == 0)
                plane, plane_t = fit_support_plane(P, usable), time.time()

            if not frozen:
                mask, cloud_mask, table = segment_object(depth_m, P, plane, max_depth, mask)
                obj = mask_to_cloud(P, color, cloud_mask)
            else:
                _, _, table = segment_object(depth_m, P, plane, max_depth)

            # --- CAD model: collect frames, then build once ------------------
            if builder is not None and not frozen:
                if cloud_mask.any():
                    builder.add(P, cloud_mask)
                if len(builder.frames) >= MODEL_FRAMES:
                    t0 = time.time()
                    mesh = builder.build()
                    builder = None
                    if mesh is None:
                        print("Model failed: no object surface found.")
                    else:
                        model = mesh
                        if renderer is None:
                            renderer = ModelRenderer((intr.width * 3 // 4, intr.height))
                        renderer.set_mesh(model)
                        show_model = True
                        path = export_stl(model)
                        ext = model.get_axis_aligned_bounding_box().get_extent() * 1000
                        print(f"Model built in {time.time() - t0:.2f}s: {ext[0]:.1f} x {ext[1]:.1f} x "
                              f"{ext[2]:.1f} mm, {len(model.triangles)} triangles -> {path}")

            # --- compose window --------------------------------------------
            left = draw_frame(color, obj, mask, table, intr, color_mode)
            panel_size = (intr.width * 3 // 4, intr.height)
            if show_depth:
                right = draw_depth(depth_m, max_depth, panel_size)
                put_text(right, ["depth (black = no data)", "d: back to 3D view"])
            elif show_model and model is not None:
                right = renderer.render(view.dx + (3.0 if view.auto else 0.0), view.dy)
                view.dx = view.dy = 0.0
                ext = model.get_axis_aligned_bounding_box().get_extent() * 1000
                put_text(right, ["CAD model - drag to rotate",
                                 f"{ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm (L x W x H)",
                                 "p: points  v: big viewer  m: rebuild"])
            else:
                if view.auto:
                    view.yaw += 0.03
                right = draw_3d_panel(obj, panel_size, view.yaw, view.pitch, color_mode)
                put_text(right, ["3D view - drag to rotate", "r: auto-rotate  d: depth"])

            pts = np.asarray(obj.points)
            no_depth = 1.0 - np.count_nonzero(depth_m > 0) / depth_m.size
            info = [f"FPS {fps:4.1f}   points {len(pts)}" + ("   [FROZEN]" if frozen else ""),
                    f"range {MIN_DEPTH:.2f}-{max_depth:.2f} m   table: {'found' if plane is not None else 'NOT found'}"]
            if len(pts):
                ext = np.sort(obj.get_axis_aligned_bounding_box().get_extent())[::-1] * 1000
                info.append(f"distance {np.median(pts[:, 2]):.2f} m   size ~{ext[0]:.0f}x{ext[1]:.0f}x{ext[2]:.0f} mm")
            if no_depth > 0.4:
                info.append(f"WARNING: {no_depth:.0%} of the image has no depth - move camera back (>0.3 m)")
            if builder is not None:
                info.append(f"BUILDING MODEL - keep still... {len(builder.frames)}/{MODEL_FRAMES}")
            elif model is None:
                info.append("press 'm' to build the CAD model")
            put_text(left, info)
            cv2.imshow(WINDOW_NAME, np.hstack((left, right)))

            n_frames += 1
            if time.time() - t_fps >= 1.0:
                fps, n_frames, t_fps = n_frames / (time.time() - t_fps), 0, time.time()
                print(f"FPS {fps:5.1f} | object points {len(pts):6d} | table {'yes' if plane is not None else 'no'}")

            # --- keys --------------------------------------------------------
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27) or cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            elif key == ord('s'):
                if len(pts):
                    os.makedirs(OUTPUT_DIR, exist_ok=True)
                    path = os.path.join(OUTPUT_DIR, f"object_{datetime.now():%Y%m%d_%H%M%S}.ply")
                    upright = o3d.geometry.PointCloud(obj)
                    upright.points = o3d.utility.Vector3dVector(pts * [1, -1, -1])    # Y up
                    o3d.io.write_point_cloud(path, upright)
                    print(f"Saved {len(pts)} points -> {path}")
                else:
                    print("No object to save.")
            elif key == ord('m'):
                if plane is None or frozen or not cloud_mask.any():
                    print("Cannot build: need the table found, an object detected and not frozen.")
                else:
                    builder = HeightMapModel(plane, P, cloud_mask)
                    print(f"Building model from {MODEL_FRAMES} frames - keep camera and part still...")
            elif key == ord('p'):
                show_model = not show_model
            elif key == ord('v') and model is not None:
                o3d.visualization.draw_geometries([flat_shaded(model)], window_name="CAD model",
                                                  mesh_show_back_face=True)
            elif key == ord('f'):
                frozen = not frozen
            elif key == ord('t'):
                plane_t = 0.0                       # forces a re-fit next frame
            elif key == ord('c'):
                color_mode = "rgb" if color_mode == "depth" else "depth"
            elif key == ord('d'):
                show_depth = not show_depth
            elif key == ord('r'):
                view.auto = not view.auto
            elif key in (ord('+'), ord('=')):
                max_depth = min(max_depth + 0.05, 3.0)
            elif key in (ord('-'), ord('_')):
                max_depth = max(max_depth - 0.05, MIN_DEPTH + 0.05)
    finally:
        pipeline.stop()
        if renderer is not None:
            renderer.close()
        cv2.destroyAllWindows()
        print("Camera stopped.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
