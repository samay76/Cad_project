"""
taught_path.py - Turn a grasp exported by the web tool into the TCP path the robot replays.

Two kinds of grasps come from the web app:

* a single saved pose (Save Grasp Pose): the path is the classic straight approach,
  pre-grasp (preGraspDistance back along TCP -Z) -> grasp, fingers fully open;
* a recording (Record button): `trajectory` = [{t, pose, width}, ...], the TCP pose relative to
  the part (mm, xyzw) and the finger opening (mm) sampled while the user moved the gripper.
  The robot follows exactly this path relative to the part, with the same finger widths.

The end of a recording where only the fingers move (the pose no longer changes) is the grip itself:
closing = 'outside' grip (fingers squeeze the outer surface), opening = 'inside' grip (fingers
expand against a bore). `gripMode` stored by the web app takes precedence over this detection.
"""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from frames import web_pose_to_T, make_T, rot_angle_deg

OPEN_WIDTH_MM = 85.0  # 2F-85; the simulator passes the selected gripper's full opening


class TaughtPath:
    def __init__(self, Ts, widths, grip_mode, pre_grip_width, recorded):
        self.Ts = Ts                       # list of T_part_tcp (m), last one = grasp pose
        self.widths = widths               # finger opening along the path (mm)
        self.grip_mode = grip_mode         # 'outside' (close to grip) or 'inside' (open to grip)
        self.pre_grip_width = pre_grip_width
        self.recorded = recorded

    @property
    def T_grasp(self):
        return self.Ts[-1]

    def __repr__(self):
        return (f'TaughtPath({len(self.Ts)} poses, {"recorded" if self.recorded else "single pose"}, '
                f'{self.grip_mode} grip, pre-grip width {self.pre_grip_width:.1f} mm)')


def _moved(Ta, Tb, mm, deg):
    return (np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]) * 1000 > mm) or rot_angle_deg(Ta[:3, :3], Tb[:3, :3]) > deg


def from_grasp(grasp, open_width_mm=OPEN_WIDTH_MM, pose_tol_mm=0.5, pose_tol_deg=0.5, keep_mm=1.5, keep_deg=1.5, keep_width=1.0):
    traj = grasp.get('trajectory') or []
    mode = grasp.get('gripMode')
    if mode not in (None, 'outside', 'inside'):
        raise ValueError(f"gripMode must be 'outside' or 'inside', got {mode!r}")

    if len(traj) < 2:
        T_grasp = web_pose_to_T(grasp['tcpPose'])
        back = make_T([0, 0, -float(grasp.get('preGraspDistance', 100.0)) / 1000.0], [1, 0, 0, 0])
        mode = mode or 'outside'
        w = open_width_mm if mode == 'outside' else 0.0
        return TaughtPath([T_grasp @ back, T_grasp], [w, w], mode, w, recorded=False)

    Ts = [web_pose_to_T(s['pose']) for s in traj]
    ws = [float(s['width']) for s in traj]

    # Grip = trailing samples at the final pose where only the width changes.
    k = len(Ts) - 1
    while k > 0 and not _moved(Ts[k - 1], Ts[-1], pose_tol_mm, pose_tol_deg):
        k -= 1
    pre_w, final_w = ws[k], ws[-1]
    if mode is None:
        mode = 'inside' if final_w > pre_w + 1.0 else 'outside'

    # Drop samples that add nothing (keeps IK fast); always keep the first and the grasp pose.
    keep_T, keep_w = [Ts[0]], [ws[0]]
    for T, w in zip(Ts[1:k + 1], ws[1:k + 1]):
        if _moved(keep_T[-1], T, keep_mm, keep_deg) or abs(w - keep_w[-1]) > keep_width:
            keep_T.append(T)
            keep_w.append(w)
    if keep_T[-1] is not Ts[k]:
        keep_T.append(Ts[k])
        keep_w.append(pre_w)
    if len(keep_T) < 2:  # recording without motion: approach from the recorded pose only
        keep_T, keep_w = [Ts[k], Ts[k]], [pre_w, pre_w]
    return TaughtPath(keep_T, keep_w, mode, pre_w, recorded=True)


def densify(Ts, widths, step_mm=4.0, step_deg=4.0):
    """Resample so consecutive poses are at most step_mm / step_deg apart (linear + slerp)."""
    out_T, out_w = [Ts[0]], [widths[0]]
    for Ta, Tb, wa, wb in zip(Ts[:-1], Ts[1:], widths[:-1], widths[1:]):
        d = np.linalg.norm(Tb[:3, 3] - Ta[:3, 3]) * 1000
        a = rot_angle_deg(Ta[:3, :3], Tb[:3, :3])
        n = max(1, int(np.ceil(max(d / step_mm, a / step_deg))))
        slerp = Slerp([0, 1], Rotation.from_matrix([Ta[:3, :3], Tb[:3, :3]]))
        for i in range(1, n + 1):
            s = i / n
            T = np.eye(4)
            T[:3, :3] = slerp(s).as_matrix()
            T[:3, 3] = (1 - s) * Ta[:3, 3] + s * Tb[:3, 3]
            out_T.append(T)
            out_w.append((1 - s) * wa + s * wb)
    return out_T, out_w


def path_length(Ts):
    """(translation m, rotation deg) summed along the path."""
    lin = sum(np.linalg.norm(b[:3, 3] - a[:3, 3]) for a, b in zip(Ts[:-1], Ts[1:]))
    ang = sum(rot_angle_deg(a[:3, :3], b[:3, :3]) for a, b in zip(Ts[:-1], Ts[1:]))
    return lin, ang
