"""
camera_feed.py - Live images of one MuJoCo camera, rendered in a separate process.

The passive MuJoCo viewer and an offscreen mujoco.Renderer cannot share one process here (both use
GLFW; together they crash), so the camera is rendered by a helper process: it loads the compiled
model (saved as .mjb), receives the simulation state (qpos + mocap) and writes

  <out_dir>/feed.jpg        latest frame, replaced atomically ~7x per second (the web app polls it)
  <out_dir>/<name>.png      full-resolution stills on request (inspection images)

Ghost-gripper geoms are hidden: the feed shows what a real camera would see.
"""
import multiprocessing as mp
import os
import queue
import time

import mujoco

FEED_SIZE = (480, 360)   # w, h
SNAP_SIZE = (640, 480)


def _worker(mjb_path, camera, out_dir, q):
    import PIL.Image
    parent = os.getppid()
    m = mujoco.MjModel.from_binary_path(mjb_path)
    os.remove(mjb_path)  # only needed to hand the model over (tens of MB per run)
    for g in range(m.ngeom):
        if m.geom(g).name.startswith('ghost_'):
            m.geom_rgba[g][3] = 0.0
    d = mujoco.MjData(m)
    feed_r = mujoco.Renderer(m, FEED_SIZE[1], FEED_SIZE[0])
    snap_r = mujoco.Renderer(m, SNAP_SIZE[1], SNAP_SIZE[0])

    def render(r, state):
        _, qpos, mpos, mquat = state[:4]
        d.qpos[:] = qpos
        d.mocap_pos[:] = mpos
        d.mocap_quat[:] = mquat
        mujoco.mj_forward(m, d)
        r.update_scene(d, camera=camera)
        return PIL.Image.fromarray(r.render())

    while True:
        try:
            msgs = [q.get(timeout=5.0)]
        except queue.Empty:
            if os.getppid() != parent:  # simulator was killed (we got re-parented): don't linger
                return
            continue
        except (EOFError, OSError):
            return
        while True:  # drain: stills are all kept, of the live frames only the newest matters
            try:
                msgs.append(q.get_nowait())
            except queue.Empty:
                break
        latest = None
        for msg in msgs:
            if msg[0] == 'snap':
                img = render(snap_r, msg)
                tmp = os.path.join(out_dir, f'.{msg[4]}.tmp.png')
                img.save(tmp)
                os.replace(tmp, os.path.join(out_dir, msg[4]))
            elif msg[0] == 'frame':
                latest = msg
            elif msg[0] == 'stop':
                return
        if latest is not None:
            tmp = os.path.join(out_dir, '.feed.tmp.jpg')
            render(feed_r, latest).save(tmp, quality=85)
            os.replace(tmp, os.path.join(out_dir, 'feed.jpg'))


class CameraFeed:
    def __init__(self, model, camera, out_dir, fps=7.0):
        self.out_dir, self.period = out_dir, 1.0 / fps
        os.makedirs(out_dir, exist_ok=True)
        mjb = os.path.join(out_dir, '.scene.mjb')
        mujoco.mj_saveModel(model, mjb, None)
        ctx = mp.get_context('spawn')
        self.q = ctx.Queue()
        self.proc = ctx.Process(target=_worker, args=(mjb, camera, out_dir, self.q), daemon=True)
        self.proc.start()
        self._last = 0.0

    @staticmethod
    def _state(data):
        return (data.qpos.copy(), data.mocap_pos.copy(), data.mocap_quat.copy())

    def push(self, data, force=False):
        """Live frame (rate-limited)."""
        now = time.time()
        if force or now - self._last >= self.period:
            self._last = now
            self.q.put(('frame', *self._state(data)))

    def snap(self, data, filename):
        """Full-resolution still saved as out_dir/filename."""
        self.q.put(('snap', *self._state(data), filename))

    def close(self, timeout=10.0):
        if self.proc.is_alive():
            self.q.put(('stop',))
            self.proc.join(timeout)
        if self.proc.is_alive():
            self.proc.terminate()
