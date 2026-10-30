"""
ik.py - Damped least-squares IK for the CS66 that reports *why* it failed.

Same algorithm as recorder.solve_ik, plus: multiple seeds, joint-limit
handling, and the final position/orientation residual so callers can raise a
clear "unreachable" error instead of silently using a bad solution.
"""
import numpy as np
import mujoco

# Same home posture as recorder.HOME_QPOS (kept here to avoid importing h5py/cv2).
HOME_QPOS = np.array([3.14159, -1.2, 1.8, -2.2, -1.57, 0.0], dtype=np.float64)
N_ARM = 6


class IKResult:
    def __init__(self, ok, q, pos_err_mm, rot_err_deg):
        self.ok, self.q, self.pos_err_mm, self.rot_err_deg = ok, q, pos_err_mm, rot_err_deg

    def __repr__(self):
        return f'IKResult(ok={self.ok}, pos_err={self.pos_err_mm:.2f} mm, rot_err={self.rot_err_deg:.2f} deg)'


def _solve_once(model, data, sid, T_target, q0, max_steps, pos_tol, rot_tol):
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    R_t = T_target[:3, :3]
    lo, hi = model.jnt_range[:N_ARM, 0], model.jnt_range[:N_ARM, 1]
    data.qpos[:N_ARM] = q0
    for _ in range(max_steps):
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        e_p = T_target[:3, 3] - data.site_xpos[sid]
        R = data.site_xmat[sid].reshape(3, 3)
        e_r = 0.5 * (np.cross(R[:, 0], R_t[:, 0]) + np.cross(R[:, 1], R_t[:, 1]) + np.cross(R[:, 2], R_t[:, 2]))
        if np.linalg.norm(e_p) < pos_tol and np.linalg.norm(e_r) < rot_tol:
            break
        mujoco.mj_jacSite(model, data, jacp, jacr, sid)
        J = np.vstack([jacp[:, :N_ARM], jacr[:, :N_ARM]])
        lam = 1e-3 + 1e-2 * np.linalg.norm(e_p)
        dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), np.concatenate([e_p, e_r]))
        data.qpos[:N_ARM] = np.clip(data.qpos[:N_ARM] + 0.6 * dq, lo, hi)
    mujoco.mj_kinematics(model, data)
    R = data.site_xmat[sid].reshape(3, 3)
    pos_err = np.linalg.norm(T_target[:3, 3] - data.site_xpos[sid]) * 1000.0
    c = (np.trace(R.T @ R_t) - 1.0) / 2.0
    rot_err = np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))
    return data.qpos[:N_ARM].copy(), pos_err, rot_err


def solve(model, data, site_name, T_target, seeds, max_steps=300, pos_tol_mm=0.5, rot_tol_deg=0.3,
          rng=None, n_random=6):
    """Tries each seed (then random seeds); returns the first converged IKResult,
    preferring the solution closest to seeds[0]. `data` is used as scratch."""
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    rng = rng or np.random.default_rng(0)
    lo, hi = model.jnt_range[:N_ARM, 0], model.jnt_range[:N_ARM, 1]
    saved = data.qpos.copy()
    all_seeds = [np.asarray(s, dtype=float) for s in seeds]
    for _ in range(n_random):
        all_seeds.append(np.clip(all_seeds[0] + rng.uniform(-1.2, 1.2, N_ARM), lo, hi))

    best = None
    for s in all_seeds:
        q, pe, re = _solve_once(model, data, sid, T_target, s, max_steps, pos_tol_mm / 1000.0,
                                np.radians(rot_tol_deg))
        ok = pe < pos_tol_mm * 2 and re < rot_tol_deg * 3
        cand = IKResult(ok, q, pe, re)
        if ok:
            # wrap continuous joints to the branch nearest the reference seed
            ref = all_seeds[0]
            two_pi = 2 * np.pi
            q_wrapped = q + two_pi * np.round((ref - q) / two_pi)
            if np.all(q_wrapped >= lo) and np.all(q_wrapped <= hi):
                cand.q = q_wrapped
            data.qpos[:] = saved
            return cand
        if best is None or (pe + re) < (best.pos_err_mm + best.rot_err_deg):
            best = cand
    data.qpos[:] = saved
    return best
