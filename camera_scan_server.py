"""
camera_scan_server.py
=====================

Runs the RealSense scan from live_object_pointcloud.py headless and serves it to the web app.
The vite dev server starts this process and forwards /api/scan/cam/* to it
(vite-plugins/mujocoBridge.ts), so the browser never talks to this port directly.

    GET  /status              JSON: camera state, object size, build progress, latest model
    GET  /stream/depth        MJPEG: colorized depth image
    GET  /stream/cloud        MJPEG: camera image with the object's point cloud painted on it
    GET  /stream/points       MJPEG: rotating 3D view of the object's point cloud
    POST /build               build the CAD model (fuses MODEL_FRAMES frames; keep camera + part still)
    POST /refit               re-fit the table plane now
    POST /depth  {"delta": m} change MAX_DEPTH (e.g. 0.05 / -0.05)
    GET  /model.stl           latest model as binary STL, millimeters, z-up, z = 0 on the table,
                              centered on the origin in X/Y

Usage:  python camera_scan_server.py [--port 8765]
"""

import argparse
import json
import os
import signal
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

import live_object_pointcloud as lop

JPEG_QUALITY = 80
STREAM_NAMES = ("depth", "cloud", "points")
RETRY_SECONDS = 2.0


def draw_depth_auto(depth_m, max_depth):
    """Colorized depth, red = near, blue = far, stretched over the depths actually in view (the fixed
    MIN..MAX range of lop.draw_depth makes a close scene one flat color). Black = no depth, gray = beyond max."""
    valid = (depth_m > lop.MIN_DEPTH) & (depth_m < max_depth)
    if np.count_nonzero(valid) > 100:
        lo, hi = np.percentile(depth_m[valid], (2, 98))
    else:
        lo, hi = lop.MIN_DEPTH, max_depth
    norm = np.clip((depth_m - lo) / max(hi - lo, 0.01), 0, 1)
    img = cv2.applyColorMap((255 * (1 - norm)).astype(np.uint8), cv2.COLORMAP_JET)
    img[depth_m <= 0] = 0
    img[depth_m >= max_depth] = (60, 60, 60)
    return img


class _CameraGone(Exception):
    pass


class Scanner:
    """Owns the camera. One background thread grabs frames, segments the object, builds the
    model on request, and publishes JPEG frames for the streams that have viewers."""

    def __init__(self):
        self.lock = threading.Lock()
        self.new_frame = threading.Condition(self.lock)
        self.jpeg = {name: None for name in STREAM_NAMES}
        self.seq = 0
        self.viewers = {name: 0 for name in STREAM_NAMES}
        self.state = {
            "camera": "starting",          # starting | streaming | error
            "message": "Starting camera...",
            "fps": 0.0,
            "points": 0,
            "table": False,
            "distanceM": None,
            "sizeMm": None,
            "maxDepthM": lop.MAX_DEPTH,
            "building": None,              # {"frames": n, "of": MODEL_FRAMES} while building
            "model": None,                 # {"id", "file", "sizeMm", "triangles"}
            "error": None,
        }
        self.model_path = None
        self._build_requested = False
        self._refit_requested = False
        self._max_depth = lop.MAX_DEPTH
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    # ---------------- commands (HTTP threads) ----------------
    def request_build(self):
        with self.lock:
            if self.state["camera"] != "streaming":
                return False, "The camera is not streaming."
            if self.state["building"] is not None:
                return False, "A model is already being built."
            self._build_requested = True
        return True, f"Building from {lop.MODEL_FRAMES} frames: keep the camera and the part still."

    def request_refit(self):
        self._refit_requested = True

    def change_depth(self, delta):
        with self.lock:
            self._max_depth = float(np.clip(self._max_depth + delta, lop.MIN_DEPTH + 0.05, 3.0))
            self.state["maxDepthM"] = round(self._max_depth, 3)

    def status(self):
        with self.lock:
            return json.loads(json.dumps(self.state))

    def wait_frame(self, name, last_seq, timeout=2.0):
        with self.new_frame:
            self.new_frame.wait_for(lambda: self.seq != last_seq or self._stop.is_set(), timeout)
            return self.seq, self.jpeg[name]

    def add_viewer(self, name, n):
        with self.lock:
            self.viewers[name] += n

    def stop(self):
        self._stop.set()
        with self.new_frame:
            self.new_frame.notify_all()

    # ---------------- camera thread ----------------
    def _set(self, **kw):
        with self.lock:
            self.state.update(kw)

    def _run(self):
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
        while not self._stop.is_set():
            try:
                self._stream()
            except (SystemExit, Exception) as err:       # lop.start_camera() calls sys.exit
                msg = str(err) or type(err).__name__
                print(f"[scan] camera error: {msg}", flush=True)
                self._set(camera="error", error=msg, message=f"Camera error: {msg} (retrying...)", building=None)
                self._stop.wait(RETRY_SECONDS)

    def _stream(self):
        pipeline, profile = lop.start_camera()
        try:
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            align = rs.align(rs.stream.color)
            filters = lop.make_filters()
            for _ in range(lop.WARMUP_FRAMES):
                pipeline.wait_for_frames()
            self._set(camera="streaming", error=None, message="Streaming")

            plane, plane_t, mask = None, 0.0, None
            builder, yaw = None, 0.0
            n_frames, t_fps = 0, time.time()

            while not self._stop.is_set():
                try:
                    frames = align.process(pipeline.wait_for_frames(5000))
                except RuntimeError as err:
                    raise _CameraGone(f"no frames from the camera ({err})")
                depth_frame, color_frame = frames.get_depth_frame(), frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue
                for f in filters:
                    depth_frame = f.process(depth_frame)
                depth_frame = depth_frame.as_depth_frame()
                intr = depth_frame.profile.as_video_stream_profile().get_intrinsics()
                depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
                color = np.asanyarray(color_frame.get_data())
                max_depth = self._max_depth

                P = lop.deproject(depth_m, intr)
                valid = (depth_m > lop.MIN_DEPTH) & (depth_m < max_depth)
                if self._refit_requested or time.time() - plane_t > lop.PLANE_REFIT_SECONDS:
                    self._refit_requested = False
                    usable = valid if mask is None else valid & (mask == 0)
                    plane, plane_t = lop.fit_support_plane(P, usable), time.time()

                mask, cloud_mask, table = lop.segment_object(depth_m, P, plane, max_depth, mask)
                obj = lop.mask_to_cloud(P, color, cloud_mask)

                # ---- CAD model ----
                if self._build_requested:
                    self._build_requested = False
                    if plane is None or not cloud_mask.any():
                        self._set(message="Cannot build: the table must be found and an object detected.")
                    else:
                        builder = lop.HeightMapModel(plane, P, cloud_mask)
                        print(f"[scan] building model from {lop.MODEL_FRAMES} frames", flush=True)
                if builder is not None:
                    if cloud_mask.any():
                        builder.add(P, cloud_mask)
                    self._set(building={"frames": len(builder.frames), "of": lop.MODEL_FRAMES},
                              message=f"Building model - keep still... {len(builder.frames)}/{lop.MODEL_FRAMES}")
                    if len(builder.frames) >= lop.MODEL_FRAMES:
                        self._finish_model(builder)
                        builder = None

                # ---- stats ----
                pts = np.asarray(obj.points)
                info = {"points": int(len(pts)), "table": plane is not None, "distanceM": None, "sizeMm": None}
                if len(pts):
                    ext = np.sort(obj.get_axis_aligned_bounding_box().get_extent())[::-1] * 1000
                    info["distanceM"] = round(float(np.median(pts[:, 2])), 3)
                    info["sizeMm"] = [round(float(e), 1) for e in ext]
                n_frames += 1
                if time.time() - t_fps >= 1.0:
                    info["fps"] = round(n_frames / (time.time() - t_fps), 1)
                    n_frames, t_fps = 0, time.time()
                self._set(**info)

                # ---- images (only for streams someone is watching) ----
                with self.lock:
                    wanted = {k for k, v in self.viewers.items() if v > 0}
                out = {}
                if "depth" in wanted:
                    out["depth"] = draw_depth_auto(depth_m, max_depth)
                if "cloud" in wanted:
                    out["cloud"] = lop.draw_frame(color, obj, mask, table, intr, "depth")
                if "points" in wanted:
                    yaw += 0.03
                    out["points"] = lop.draw_3d_panel(obj, (intr.width, intr.height), yaw, -0.5, "rgb")
                encoded = {}
                for name, img in out.items():
                    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    if ok:
                        encoded[name] = buf.tobytes()
                with self.new_frame:
                    self.jpeg.update(encoded)
                    self.seq += 1
                    self.new_frame.notify_all()
        finally:
            pipeline.stop()

    def _finish_model(self, builder):
        t0 = time.time()
        mesh = builder.build()
        if mesh is None:
            self._set(building=None, message="Model failed: no object surface found. Try again.")
            return
        # Center in X/Y (z = 0 stays on the table) so the part lands at the viewer's origin.
        bb = mesh.get_axis_aligned_bounding_box()
        c = bb.get_center()
        mesh.translate((-c[0], -c[1], 0.0))
        out = o3d.geometry.TriangleMesh(mesh)
        out.scale(1000.0, center=np.zeros(3))
        out.compute_triangle_normals()
        os.makedirs(lop.OUTPUT_DIR, exist_ok=True)
        model_id = f"{datetime.now():%Y%m%d_%H%M%S}"
        path = os.path.join(lop.OUTPUT_DIR, f"scan_{model_id}.stl")
        o3d.io.write_triangle_mesh(path, out)
        ext = bb.get_extent() * 1000
        self.model_path = path
        print(f"[scan] model built in {time.time() - t0:.2f}s: {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm, "
              f"{len(out.triangles)} triangles -> {path}", flush=True)
        self._set(building=None,
                  message="Model ready.",
                  model={"id": model_id, "file": os.path.basename(path),
                         "sizeMm": [round(float(e), 1) for e in ext], "triangles": len(out.triangles)})


def make_handler(scanner):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_):
            pass

        def _json(self, code, body):
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/status":
                return self._json(200, scanner.status())
            if path.startswith("/stream/") and path[len("/stream/"):] in STREAM_NAMES:
                return self._mjpeg(path[len("/stream/"):])
            if path == "/model.stl":
                p = scanner.model_path
                if not p or not os.path.exists(p):
                    return self._json(404, {"ok": False, "message": "No model built yet."})
                with open(p, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "model/stl")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
                return
            self._json(404, {"ok": False, "message": "Not found"})

        def do_POST(self):
            path = self.path.split("?")[0]
            length = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(length) or b"{}") if length else {}
            except ValueError:
                return self._json(400, {"ok": False, "message": "Bad JSON"})
            if path == "/build":
                ok, msg = scanner.request_build()
                return self._json(200 if ok else 409, {"ok": ok, "message": msg})
            if path == "/refit":
                scanner.request_refit()
                return self._json(200, {"ok": True})
            if path == "/depth":
                delta = body.get("delta")
                if not isinstance(delta, (int, float)) or abs(delta) > 1:
                    return self._json(400, {"ok": False, "message": "delta must be a number of meters"})
                scanner.change_depth(float(delta))
                return self._json(200, {"ok": True, "maxDepthM": scanner.status()["maxDepthM"]})
            self._json(404, {"ok": False, "message": "Not found"})

        def _mjpeg(self, name):
            self.close_connection = True
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            scanner.add_viewer(name, 1)
            seq = -1
            try:
                while not scanner._stop.is_set():
                    seq, jpg = scanner.wait_frame(name, seq)
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                scanner.add_viewer(name, -1)

    return Handler


class QuietServer(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        # Browsers / the dev-server bridge drop connections (closed stream, reused socket): not an error.
        if isinstance(sys.exc_info()[1], (ConnectionResetError, BrokenPipeError, ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()

    scanner = Scanner()
    scanner.thread.start()
    server = QuietServer(("127.0.0.1", args.port), make_handler(scanner))
    # The dev server stops us with SIGTERM: shut down cleanly so the camera is released.
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    print(f"[scan] camera scan server on http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        scanner.stop()
        server.server_close()
        scanner.thread.join(timeout=5)
        print("[scan] stopped", flush=True)


if __name__ == "__main__":
    main()
