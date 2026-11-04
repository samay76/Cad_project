"""
motion_planner.py - Collision-free arm motion: straight TCP lines and joint-space RRT-Connect.

The planner knows nothing about grasps. It is given a `free(q) -> bool` test for each motion, so the
caller decides what counts as a collision (fingers open or closed, part on the table or carried).
"""
import time

import numpy as np

from ik import solve as ik_solve, N_ARM

LINE_STEP_M = 0.004       # spacing of IK poses along a straight TCP line
MAX_JOINT_JUMP = 0.5      # rad between consecutive line poses; more means an arm reconfiguration
EDGE_STEP = 0.025         # rad between collision checks along a joint-space segment
RRT_STEP = 0.35           # rad, RRT growth step
DENSE_STEP = 0.03         # rad between waypoints handed to the controller


class MotionError(Exception):
    """No motion found. The message says why."""


def joint_path_length(path):
    q = np.asarray(path)
    return float(np.abs(np.diff(q, axis=0)).max(axis=1).sum()) if len(q) > 1 else 0.0


def _interpolate(qa, qb, step):
    n = max(1, int(np.ceil(np.max(np.abs(qb - qa)) / step)))
    return [qa + (qb - qa) * i / n for i in range(1, n + 1)]


def segment_free(qa, qb, free, step=EDGE_STEP):
    return all(free(q) for q in _interpolate(np.asarray(qa), np.asarray(qb), step))


def resample(path, step=DENSE_STEP):
    """Uniformly spaced waypoints: the controller gives each waypoint the same time."""
    out = [np.asarray(path[0], dtype=float)]
    for qb in path[1:]:
        out.extend(_interpolate(out[-1], np.asarray(qb, dtype=float), step))
    return out


class MotionPlanner:
    def __init__(self, model, scratch, tcp_site, seed=0):
        self.model, self.scratch, self.tcp_site = model, scratch, tcp_site
        self.rng = np.random.default_rng(seed)
        self.lo = model.jnt_range[:N_ARM, 0].copy()
        self.hi = model.jnt_range[:N_ARM, 1].copy()

    def ik(self, T, seeds):
        """Joint solution for a TCP pose, nearest the first seed, or None."""
        r = ik_solve(self.model, self.scratch, self.tcp_site, T, seeds)
        return r.q if r.ok else None

    def line(self, label, T_a, T_b, q_start, free):
        """Joint path along a straight, constant-orientation TCP line from T_a (already at q_start) to T_b."""
        dist = np.linalg.norm(T_b[:3, 3] - T_a[:3, 3])
        n = max(1, int(np.ceil(dist / LINE_STEP_M)))
        path, q = [], np.asarray(q_start, dtype=float)
        for i in range(1, n + 1):
            T = T_b.copy()
            T[:3, 3] = T_a[:3, 3] + (T_b[:3, 3] - T_a[:3, 3]) * i / n
            q_new = self.ik(T, [q])
            if q_new is None:
                raise MotionError(f"'{label}': the arm cannot follow the straight line ({i}/{n}).")
            if np.max(np.abs(q_new - q)) > MAX_JOINT_JUMP:
                raise MotionError(f"'{label}': the straight line needs an arm reconfiguration ({i}/{n}).")
            if not segment_free(q, q_new, free):
                raise MotionError(f"'{label}': the straight line collides ({i}/{n}).")
            path.append(q_new)
            q = q_new
        return path

    def connect(self, label, qa, qb, free, time_limit=6.0):
        """Collision-free joint path from qa to qb (first and last waypoint included)."""
        qa, qb = np.asarray(qa, dtype=float), np.asarray(qb, dtype=float)
        if not free(qa):
            raise MotionError(f"'{label}': the start posture is in collision.")
        if not free(qb):
            raise MotionError(f"'{label}': the goal posture is in collision.")
        if segment_free(qa, qb, free):
            return resample([qa, qb])
        path = self._rrt_connect(label, qa, qb, free, time_limit)
        path = self._shortcut(path, free)
        return self._smooth(resample(path), free)

    # ---------------- RRT-Connect ----------------
    def _sample(self, qa, qb):
        # Stay near the two postures: joints have +-360 deg of range the arm should not wander through.
        lo = np.maximum(self.lo, np.minimum(qa, qb) - 1.8)
        hi = np.minimum(self.hi, np.maximum(qa, qb) + 1.8)
        return self.rng.uniform(lo, hi)

    @staticmethod
    def _nearest(tree, q):
        return int(np.argmin(np.linalg.norm(np.asarray(tree['q']) - q, axis=1)))

    def _extend(self, tree, q_target, free, greedy):
        """Grows `tree` towards q_target. Returns (reached, index of the last node added)."""
        i = self._nearest(tree, q_target)
        while True:
            q_near = tree['q'][i]
            delta = q_target - q_near
            dist = np.linalg.norm(delta)
            reached = dist <= RRT_STEP
            q_new = q_target if reached else q_near + delta / dist * RRT_STEP
            if not segment_free(q_near, q_new, free):
                return False, i
            tree['q'].append(q_new)
            tree['parent'].append(i)
            i = len(tree['q']) - 1
            if reached:
                return True, i
            if not greedy:
                return False, i

    def _rrt_connect(self, label, qa, qb, free, time_limit):
        ta = {'q': [qa], 'parent': [-1]}
        tb = {'q': [qb], 'parent': [-1]}
        start = time.time()
        swapped = False
        while time.time() - start < time_limit:
            q_rand = qb if self.rng.random() < 0.1 and not swapped else self._sample(qa, qb)
            _, ia = self._extend(ta, q_rand, free, greedy=False)
            reached, ib = self._extend(tb, ta['q'][ia], free, greedy=True)
            if reached:
                pa = self._trace(ta, ia)
                pb = self._trace(tb, ib)
                path = pa + pb[::-1][1:]
                return path[::-1] if swapped else path
            ta, tb = tb, ta
            swapped = not swapped
        raise MotionError(f"'{label}': no collision-free path found in {time_limit:.0f} s.")

    @staticmethod
    def _trace(tree, i):
        path = []
        while i >= 0:
            path.append(tree['q'][i])
            i = tree['parent'][i]
        return path[::-1]

    # ---------------- path cleanup ----------------
    def _shortcut(self, path, free, rounds=60):
        path = list(path)
        for _ in range(rounds):
            if len(path) < 3:
                break
            i, j = sorted(self.rng.choice(len(path), 2, replace=False))
            if j - i >= 2 and segment_free(path[i], path[j], free):
                path = path[:i + 1] + path[j:]
        return path

    @staticmethod
    def _smooth(dense, free, half_window=8):
        """Rounds the corners that would jerk the arm. Keeps the raw path if the rounded one collides."""
        if len(dense) < 2 * half_window + 2:
            return dense
        q = np.asarray(dense)
        kernel = np.hanning(2 * half_window + 3)[1:-1]
        kernel /= kernel.sum()
        pad = np.vstack([np.repeat(q[:1], half_window, axis=0), q, np.repeat(q[-1:], half_window, axis=0)])
        smooth = np.column_stack([np.convolve(pad[:, k], kernel, mode='valid') for k in range(q.shape[1])])
        smooth[0], smooth[-1] = q[0], q[-1]
        candidate = resample(list(smooth))
        if all(segment_free(a, b, free) for a, b in zip(candidate[:-1], candidate[1:])):
            return candidate
        return dense
