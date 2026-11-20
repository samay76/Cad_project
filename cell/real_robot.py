"""
real_robot.py - Runs a planned pick & place on the real Elite EC66, with the MuJoCo scene following it live.

Used by `auto_pick_place.py --mode real` (the web app's "Real Sim" button). The robot side is taken from the
the former standalone tester ec66_sim.py:

  * JSON-RPC client on port 8055 (getRobotPos, moveByJoint, stop, ...). Motion needs REMOTE mode.
  * One moveByJoint per waypoint at a slow joint speed (default 5 %, hard cap 10 %), waiting until the robot
    has arrived before sending the next one. The sim mirrors the real joints at 20 Hz while it moves.
  * Ctrl+C / SIGTERM (the web app's Stop button) / closing the MuJoCo window sends `stop` to the robot.

The web planner's EC66 model has the same joint zeros as the real controller (checked against the standalone
model: identical flange pose for the same joints), so the plan's joints go to the robot as they are, in degrees.

The planned joint paths are dense (straight TCP lines and RRT paths sampled finely). They are reduced to the
fewest waypoints whose joint-linear moves (what moveByJoint does) keep the TCP within WAYPOINT_TOL_M of the
planned path, so the robot stops as few times as possible.

The gripper: with the real 2F-85 enabled (config/cell.json real_gripper, the web app's "Real gripper" box) the
grip / release phases drive it over USB (real_gripper.py) and the sim fingers follow it; otherwise they are only
shown in the sim. The arm holds still while the gripper moves.

ip 'fake' runs everything against a simulated robot (FakeEC66), for testing without hardware.
"""
import json
import math
import signal
import socket
import time

import mujoco
import numpy as np

from .cad_pick_place import PlanError, TCP_SITE
from .real_gripper import FakeGripper, RealGripper, pos_to_width, set_closed_pos, width_to_pos
from .frames import site_T
from .ik import N_ARM

POLL_HZ = 20
REAL_SPEED_DEFAULT = 5          # % joint speed
REAL_SPEED_MAX = 10             # hard cap: slow motion only
FAKE_START_DEG = (197.7, -74.4, -119.0, -77.0, -271.1, 17.7)   # fake robot's start for Go Home (a real pose)
SPLIT_DEG = 90.0                # Real Sim: longer joint moves are split into shorter ones (same path)
MAX_JOINT_SWING_DEG = 120       # refuse a single move that turns a joint more than this
START_TOLERANCE_DEG = 0.5       # the robot must not move between reading its joints and starting
ARRIVE_TOLERANCE_DEG = 1.0      # max joint error accepted at a waypoint
WAYPOINT_TOL_M = 0.003          # TCP deviation allowed between the reduced and the planned path
NEAR_PART_TOL_M = 0.0005        # ... in the phases near the part (approach, lift, place, retreat): 0.5 mm
NEAR_PART_PHASES = ('approach', 'lift', 'place', 'retreat')
BLEND_MM = 10.0                 # smooth motion: corners between path points are rounded by up to this much (mm)
NEAR_PART_BLEND_MM = 1.0        # ... near the part (approach, lift, place, retreat)
JOINT_TOL_DEG = 2.0             # ... and joint deviation (keeps the wrist from swinging differently)
WRIST_TABLE_MARGIN = 0.015      # wrist (link5) stays this far above the table top (m)
PENDANT_MIN = (0.686, -0.486, 0.0)   # teach-pendant holder (main-sim world frame, table top z=0), from ec66_sim.py
PENDANT_MAX = (0.845, -0.193, 0.302)
PENDANT_MARGIN = 0.03


# --------------------------------------------------------------------------------------------- robots
class EC66:
    """Elite EC66 JSON-RPC client (port 8055). Reading is always safe; motion needs REMOTE mode."""

    def __init__(self, ip, port=8055, timeout=5.0, src=None):
        source = (src, 0) if src else None
        self.sock = socket.create_connection((ip, port), timeout=timeout, source_address=source)
        self.buf = b""
        self._id = 0

    def close(self):
        self.sock.close()

    def call(self, method, params=None):
        self._id += 1
        req = {"jsonrpc": "2.0", "method": method, "params": params or {}, "id": self._id}
        self.sock.sendall((json.dumps(req) + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("robot closed the connection")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        reply = json.loads(line)
        if "error" in reply:
            raise RuntimeError(f"{method} failed: {reply['error']}")
        result = reply.get("result")
        if isinstance(result, str):  # the controller returns results as JSON strings
            try:
                result = json.loads(result)
            except json.JSONDecodeError:
                pass
        return result

    def joints(self):
        return [float(v) for v in self.call("getRobotPos")[:6]]

    def mode(self):
        return {0: "TEACH", 1: "PLAY", 2: "REMOTE"}.get(self.call("getRobotMode"), "?")

    def state(self):
        return {0: "stop", 1: "pause", 2: "e-stop", 3: "running", 4: "alarm", 5: "collision"}.get(
            self.call("getRobotState"), "?")

    def servo_on(self):
        if not self.call("getServoStatus"):
            self.call("set_servo_status", {"status": 1})
            time.sleep(1.0)
        if not self.call("getServoStatus"):
            raise RuntimeError("could not switch the servos on")

    def move_joints(self, deg, speed_pct):
        return self.call("moveByJoint", {"targetPos": list(map(float, deg)), "speed": float(speed_pct)})

    def move_path(self, points_deg, speed_pct, blend_mm):
        """One smooth motion through all points (the controller's path: clearPathPoint, addPathPoint, moveByPath).
        blend_mm[i]: how far before point i the robot may round the corner into the next segment (0 = pass exactly
        through it); the last point is always reached exactly. Older firmware takes a 0..7 'smooth' level instead
        of 'circular_radius': that is tried next. Returns False if the controller has no path motion."""
        for style in ('radius', 'smooth'):
            try:
                self.call("clearPathPoint")
                for k, (q, r) in enumerate(zip(points_deg, blend_mm)):
                    last = k == len(points_deg) - 1
                    p = {"wayPoint": list(map(float, q)), "moveType": 0, "speed": float(speed_pct)}
                    if style == 'radius':
                        p["circular_radius"] = 0.0 if last else float(r)
                    else:
                        p["smooth"] = 0 if last else int(min(7, max(0, round(r / 5.0))))
                    if self.call("addPathPoint", p) is False:
                        raise RuntimeError("addPathPoint returned False")
                if self.call("moveByPath") in (False, -1):
                    raise RuntimeError("moveByPath refused the path")
                return True
            except RuntimeError as e:
                print(f"[real] path motion ({style}) not accepted: {e}", flush=True)
        try:
            self.call("clearPathPoint")
        except RuntimeError:
            pass
        return False

    def stop(self):
        try:
            self.call("stop")
        except Exception as e:  # noqa: BLE001 - best effort
            print(f"[real] stop failed: {e}", flush=True)


class FakeEC66:
    """Stands in for the robot (ip 'fake'): joint-linear moves at ~150 deg/s x speed %, like the real one."""

    def __init__(self, start_deg):
        self.q = np.asarray(start_deg, dtype=float)
        self.target, self.t0, self.dur, self.q0 = None, 0.0, 0.0, self.q

    def _update(self):
        if self.target is None:
            return
        s = min(1.0, (time.time() - self.t0) / self.dur)
        self.q = self.q0 + (0.5 - 0.5 * math.cos(math.pi * s)) * (self.target - self.q0)
        if s >= 1.0:
            self.target = None
            if getattr(self, 'path', None):
                self._next()

    def joints(self):
        self._update()
        return self.q.tolist()

    def move_path(self, points_deg, speed_pct, blend_mm):
        self._update()
        self.path = [np.asarray(q, dtype=float) for q in points_deg]
        self.path_speed = speed_pct
        self._next()
        return True

    def _next(self):
        q = self.path.pop(0)
        self.q0, self.target, self.t0 = self.q.copy(), q, time.time()
        self.dur = max(0.05, float(np.max(np.abs(self.target - self.q0))) / (150.0 * self.path_speed / 100.0))

    def mode(self):
        return "REMOTE"

    def state(self):
        self._update()
        return "running" if self.target is not None else "stop"

    def servo_on(self):
        pass

    def move_joints(self, deg, speed_pct):
        self._update()
        self.path = []
        self.q0, self.target, self.t0 = self.q.copy(), np.asarray(deg, dtype=float), time.time()
        self.dur = max(0.2, float(np.max(np.abs(self.target - self.q0))) / (150.0 * speed_pct / 100.0))
        return True

    def stop(self):
        self._update()
        self.target = None

    def close(self):
        pass


# ------------------------------------------------------------------------------------------ waypoints
def _fk(model, data, q):
    data.qpos[:N_ARM] = q
    mujoco.mj_kinematics(model, data)
    return site_T(model, data, TCP_SITE)[:3, 3].copy()


def reduce_path(model, data, qs, tol_m=WAYPOINT_TOL_M, tol_deg=JOINT_TOL_DEG):
    """Indices of the fewest samples of qs such that joint-linear moves between them stay within tol of qs."""
    qs = np.asarray(qs, dtype=float)
    if len(qs) <= 2:
        return list(range(len(qs)))
    tcp = np.array([_fk(model, data, q) for q in qs])
    tol_rad = math.radians(tol_deg)
    keep, i = [0], 0
    while i < len(qs) - 1:
        j = i + 1
        while j + 1 < len(qs) and _segment_ok(model, data, qs, tcp, i, j + 1, tol_m, tol_rad):
            j += 1
        keep.append(j)
        i = j
    return keep


def _segment_ok(model, data, qs, tcp, i, j, tol_m, tol_rad):
    a, b = qs[i], qs[j]
    for k in range(i + 1, j):
        # The planned sample's position along the move, by joint-path length.
        s = np.linalg.norm(qs[k] - a) / max(np.linalg.norm(b - a), 1e-9)
        q = a + min(s, 1.0) * (b - a)
        if np.max(np.abs(q - qs[k])) > tol_rad or np.linalg.norm(_fk(model, data, q) - tcp[k]) > tol_m:
            return False
    # Also between samples: the joint-linear move must stay near the planned polyline.
    for s in np.linspace(0, 1, 13)[1:-1]:
        p = _fk(model, data, a + s * (b - a))
        if _dist_to_polyline(p, tcp[i:j + 1]) > tol_m:
            return False
    return True


def _dist_to_polyline(p, pts):
    best = np.min(np.linalg.norm(pts - p, axis=1))
    for a, b in zip(pts, pts[1:]):
        ab = b - a
        t = np.clip((p - a) @ ab / max(ab @ ab, 1e-12), 0.0, 1.0)
        best = min(best, np.linalg.norm(a + t * ab - p))
    return best


def real_waypoints(model, data, plan):
    """Steps for the real robot: {'kind': 'move', 'q' (rad)} and {'kind': 'grip'} (gripper command), each with the
    plan phase's 'phase', 'label', 'width' (mm) and 'attached'.

    Gripper steps: 'grip' squeezes the part (outside grip: close until contact; inside: open until contact),
    'release' opens to the release width, and any other phase that starts with a different finger opening while
    nothing is carried (e.g. opening fully before going home) first sets that opening."""
    out = []
    width = plan['phases'][0]['width']
    for phase in plan['phases']:
        info = {'phase': phase['name'], 'label': phase['label'], 'attached': phase['attached']}
        if phase['name'] in ('wait', 'hold', 'look'):   # stand still for this long (grasp test, after the grip, camera)
            out.append({**info, 'kind': 'wait', 'q': None, 'width': None, 'seconds': float(phase['seconds'])})
            continue
        if phase['name'] in ('grip', 'release'):
            grip_w = (plan['widthMm'] if plan['mode'] == 'outside' else
                      next(p['width'] for p in plan['phases'] if p['name'] in ('lift', 'hold')))
            target = grip_w if phase['name'] == 'grip' else phase['width']
            out.append({**info, 'kind': 'grip', 'q': None, 'width': target, 'attached': phase['name'] == 'grip',
                        'squeeze': phase['name'] == 'grip', 'inside': plan['mode'] == 'inside'})
            width = target
            continue
        if not phase['attached'] and abs(phase['width'] - width) > 1.0:
            out.append({**info, 'kind': 'grip', 'q': None, 'width': phase['width'], 'squeeze': False,
                        'label': f"Fingers to {phase['width']:.0f} mm before: {phase['label']}"})
            width = phase['width']
        qs = np.asarray(phase['q'], dtype=float)
        tol = NEAR_PART_TOL_M if phase['name'] in NEAR_PART_PHASES else WAYPOINT_TOL_M
        for k in reduce_path(model, data, qs, tol_m=tol):
            out.append({**info, 'kind': 'move', 'q': qs[k], 'width': width})
    # Drop moves that do not move the arm; split long ones (e.g. the wrist turning in place at home) into
    # shorter ones on the same joint-linear path, so no single moveByJoint turns a joint more than SPLIT_DEG.
    steps, last = [], None
    for w in out:
        if w['kind'] != 'move':
            steps.append(w)
            continue
        if last is not None:
            big = math.degrees(float(np.max(np.abs(w['q'] - last))))
            if big < 0.05:
                continue
            n = int(math.ceil(big / SPLIT_DEG))
            for i in range(1, n):
                steps.append({**w, 'q': last + (w['q'] - last) * i / n})
        steps.append(w)
        last = w['q']
    return steps


def smooth_paths(steps):
    """Joins consecutive moves into one smooth path ({'kind': 'path', 'moves': [...]}). A path ends (the robot stops
    exactly there) before anything that is not a move (gripper, wait) and where the motion goes into or out of a
    phase near the part, so the robot comes smoothly to just above the part, stops, goes straight down, and so on."""
    out = []
    for w in steps:
        if w['kind'] != 'move':
            out.append(w)
            continue
        prev = out[-1] if out and out[-1]['kind'] == 'path' else None
        last = prev['moves'][-1] if prev else None
        if prev is not None and (last['phase'] == w['phase'] or
                                 (last['phase'] not in NEAR_PART_PHASES and w['phase'] not in NEAR_PART_PHASES)):
            prev['moves'].append(w)
        else:
            out.append({'kind': 'path', 'phase': w['phase'], 'label': w['label'], 'moves': [w]})
    return out


def moves_of(steps):
    return [w for w in steps if w['kind'] == 'move']


def match_turns(wps, start_deg, lo, hi):
    """Shifts each joint of the whole plan by whole turns (360 deg) towards the turn the robot is on now.
    -271 deg and 89 deg are the same wrist pose; without this the robot would spin a full turn first.
    A shift is only made when every waypoint stays inside the joint limits. Returns what was shifted."""
    start = np.radians(np.asarray(start_deg, dtype=float))
    wps = moves_of(wps)
    first = wps[0]['q']
    done = []
    for j in range(len(start)):
        k = round((start[j] - first[j]) / (2 * np.pi))
        if k == 0:
            continue
        shift = 2 * np.pi * k
        if all(lo[j] - 1e-6 <= w['q'][j] + shift <= hi[j] + 1e-6 for w in wps):
            for w in wps:
                w['q'] = w['q'].copy()
                w['q'][j] += shift
            done.append(f'J{j + 1} {360 * k:+d} deg')
    return done


UPRIGHT_DEG = (-90.0, 0.0, -90.0)   # J2, J3, J4 with the arm pointing straight up (flange ~1 m above the base)
HOME_SPLIT_DEG = 100.0               # Go Home: no single move turns a joint more than this


def home_route(start_deg, home_q):
    """Go Home waypoints from anywhere: raise the arm straight up in place, turn J1 and the wrist while it is
    upright (high above the table), then lower into the home pose. Long moves are split into shorter ones."""
    start = np.radians(np.asarray(start_deg, dtype=float))
    up = np.radians(UPRIGHT_DEG)
    raised = start.copy()
    raised[1:4] = up
    turned = np.asarray(home_q, dtype=float).copy()
    turned[1:4] = up
    route = []
    prev = start
    for name, label, q in (('raise', 'Raising the arm straight up', raised),
                           ('turn', 'Turning base and wrist to the home direction', turned),
                           ('lower', 'Lowering into the home pose', np.asarray(home_q, dtype=float))):
        n = max(1, int(math.ceil(math.degrees(float(np.max(np.abs(q - prev)))) / HOME_SPLIT_DEG)))
        for i in range(1, n + 1):
            route.append({'phase': name, 'label': label, 'q': prev + (q - prev) * i / n,
                          'width': None, 'attached': False, 'step': i})
        prev = q
    # skip moves that do not move anything (e.g. already upright)
    out, last = [], start
    for w in route:
        if np.max(np.abs(w['q'] - last)) > math.radians(0.05):
            out.append(w)
            last = w['q']
    return out


def check_real_path(model, data, start_deg, wps, lo, hi, samples=40):
    """Problems with the joint moves start -> waypoints, as moveByJoint would make them."""
    problems = []
    wrist = model.body('link5').id
    tcp = model.site(TCP_SITE).id
    pmin = np.array(PENDANT_MIN) - PENDANT_MARGIN
    pmax = np.array(PENDANT_MAX) + PENDANT_MARGIN
    prev, prev_name = np.radians(start_deg), 'current position'
    for w in moves_of(wps):
        q = w['q']
        if np.any(q < lo - 1e-6) or np.any(q > hi + 1e-6):
            problems.append(f"{w['phase']}: joint limit exceeded")
        swing = math.degrees(float(np.max(np.abs(q - prev))))
        if swing > MAX_JOINT_SWING_DEG:
            problems.append(f"{prev_name} -> {w['phase']}: a joint would turn {swing:.0f} deg "
                            f"(> {MAX_JOINT_SWING_DEG}); the robot is too far from the home pose")
        for s in np.linspace(0, 1, samples):
            data.qpos[:N_ARM] = prev + s * (q - prev)
            mujoco.mj_kinematics(model, data)
            if data.xpos[wrist][2] < WRIST_TABLE_MARGIN:
                problems.append(f"{prev_name} -> {w['phase']}: wrist {data.xpos[wrist][2] * 1000:.0f} mm "
                                f"above the table")
                break
            if any(np.all(p >= pmin) and np.all(p <= pmax) for p in (data.xpos[wrist], data.site_xpos[tcp])):
                problems.append(f"{prev_name} -> {w['phase']}: comes within {PENDANT_MARGIN * 1000:.0f} mm "
                                f"of the teach-pendant holder")
                break
        prev, prev_name = q, w['phase']
    return problems


def local_candidates(ip):
    """Local IPv4 addresses on the robot's /24 (e.g. 192.168.1.100 on the robot's Ethernet cable), best first.
    A laptop on the same subnet over Wi-Fi too would otherwise talk to the robot from the wrong address."""
    import subprocess
    try:
        out = subprocess.run(['ip', '-4', '-o', 'addr'], capture_output=True, text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    prefix = ip.rsplit('.', 1)[0] + '.'
    found = []
    for line in out.splitlines():
        parts = line.split()
        if 'inet' in parts:
            addr, bits = parts[parts.index('inet') + 1].split('/')
            if addr.startswith(prefix) and addr != ip:
                found.append((bits != '32', addr))   # a /32 address is one added just for the robot: try first
    return [a for _, a in sorted(found)]


def connect(ip, src=None):
    """EC66 client. Without src: the default route first, then each local address on the robot's subnet."""
    if src:
        return EC66(ip, src=src)
    tries = [None] + local_candidates(ip)
    last = None
    for i, s in enumerate(tries):
        try:
            robot = EC66(ip, src=s, timeout=2.0 if i < len(tries) - 1 else 5.0)
            robot.sock.settimeout(5.0)
            if s:
                print(f'[real] connected to {ip} from {s}', flush=True)
            return robot
        except OSError as e:
            last = e
    raise last


# ------------------------------------------------------------------------------------------------ run
class Stopped(Exception):
    pass


def run_on_real_robot(sim, doc, plan, ip, src=None, speed=REAL_SPEED_DEFAULT, go_home=False, gripper_cfg=None,
                      pause_at_pick_s=0.0, smooth_motion=True):
    """Moves the real robot through the plan (or, go_home, only to the plan's home pose by home_route);
    `sim` (AutoPickPlace) mirrors it. gripper_cfg (config/cell.json real_gripper) drives the real 2F-85 on USB;
    None = the gripper only moves in the sim. Returns a summary."""
    st = sim.status
    if doc.get('robot') != 'ec66':
        raise PlanError(f"Real Sim drives the Elite EC66, but this plan is for '{doc.get('robot')}'. "
                        'Choose the EC66 and plan again.')
    speed = max(1.0, min(float(speed), REAL_SPEED_MAX))
    m, d, s = sim.model, sim.data, sim.scratch
    lo, hi = m.jnt_range[:N_ARM, 0], m.jnt_range[:N_ARM, 1]

    sim.phase('connecting', f'Connecting to the robot at {ip}...')
    fake = ip == 'fake'
    try:
        robot = FakeEC66(FAKE_START_DEG if go_home else np.degrees(plan['phases'][0]['q'][0])) if fake \
        else connect(ip, src)
    except OSError as e:
        raise PlanError(f'Could not connect to the robot at {ip}:8055 ({e}). Check the IP, the cable and that the '
                        'controller is on. The robot was not moved.') from e
    stop_requested = {'why': None}

    def on_signal(signum, _frame):
        stop_requested['why'] = 'Stopped from the web app.'
        raise Stopped()

    old_term = signal.signal(signal.SIGTERM, on_signal)
    moving = False
    gripper = None
    try:
        mode, state = robot.mode(), robot.state()
        st.info['real_robot'] = {'ip': ip, 'mode': mode, 'state': state, 'speed_pct': speed, 'fake': fake}
        if mode != 'REMOTE':
            raise PlanError(f'The robot is in {mode} mode. Switch the teach pendant to REMOTE mode, then press Real Sim again.')
        if state != 'stop':
            raise PlanError(f"The robot state is '{state}'. Clear alarms / stop it on the pendant first.")

        start = np.array(robot.joints())
        wps = real_waypoints(m, s, plan)
        turned = match_turns(wps, start, lo, hi)
        if turned:
            st.info['real_turns'] = turned
            print('[real] plan shifted by whole turns to match the robot: ' + ', '.join(turned), flush=True)
        if go_home:
            wps = home_route(start, moves_of(wps)[0]['q'])
            for w in wps:
                w['kind'] = 'move'
            if not wps:
                return 'The robot is already at the home pose. Press Real Sim.'
        problems = check_real_path(m, s, start, wps, lo, hi)
        if problems:
            hint = '' if go_home else ' Press Go Home first to bring the robot to the home pose.'
            raise PlanError('Real move refused (the robot was not moved):\n  ' + '\n  '.join(problems[:6]) + hint)
        st.info['real_waypoints'] = len(moves_of(wps))
        if gripper_cfg is not None and not go_home and doc.get('gripper') != '2f85':
            raise PlanError(f"The real gripper is a Robotiq 2F-85, but this plan is for '{doc.get('gripper')}'.")

        T_wp = np.array(doc['part']['T_world_part'])
        T_tcp_part = np.array(plan['T_tcp_part'])
        sim.open_viewer()
        sim.show_path(sim._world_path(plan))
        sim._set_part_qpos(T_wp)
        width = {'now': plan['phases'][0]['width']}
        carried = {'on': False}
        if gripper_cfg is not None:
            set_closed_pos(gripper_cfg.get('closed_pos', 228))   # the 2F-85's real 0 mm position
        if gripper_cfg is not None and not go_home:   # Go Home does not need the gripper
            sim.phase('gripper', 'Connecting to the gripper on ' + gripper_cfg.get('port', '/dev/ttyUSB0') + '...')
            gripper = FakeGripper(width_to_pos(width['now'])) if fake else RealGripper(
                gripper_cfg.get('port', '/dev/ttyUSB0'), int(gripper_cfg.get('baud', 115200)),
                int(gripper_cfg.get('slave', 9)))
            st.info['real_gripper'] = {'port': gripper_cfg.get('port'), 'fake': fake}

        t_view0 = time.time()

        def mirror(deg):
            d.qpos[:N_ARM] = np.radians(deg)
            if gripper is not None:
                width['now'] = pos_to_width(gripper.position())
            sim.fingers.set_qpos(d.qpos, width['now'])
            mujoco.mj_kinematics(m, d)
            if carried['on']:
                sim._set_part_qpos(site_T(m, d, TCP_SITE) @ T_tcp_part)
            d.qvel[:] = 0
            mujoco.mj_forward(m, d)
            if sim.web is not None:
                sim.web.frame(d, time.time() - t_view0)
            if sim.viewer is not None:
                if not sim.viewer.is_running():
                    stop_requested['why'] = 'The MuJoCo window was closed.'
                    raise Stopped()
                sim.viewer.sync()

        mirror(start)
        if gripper is not None:
            g_speed, g_force = int(gripper_cfg.get('speed', 64)), int(gripper_cfg.get('force', 50))
            if not gripper.activated():
                sim.phase('gripper', 'Activating the gripper (the fingers close and open once)...')
                gripper.activate()
            _grip(gripper, width_to_pos(plan['phases'][0]['width']), g_speed, g_force, robot, mirror, 'start opening')
        robot.servo_on()
        moving = True
        t_begin = time.time()
        n_moves, k, grip_notes = len(moves_of(wps)), 0, []
        grasp_test = bool(plan.get('graspTest'))
        if grasp_test:
            pause_at_pick_s = 0.0   # the plan has its own wait at the pick point
        smooth = {'on': bool(smooth_motion)}
        st.info['real_smooth'] = smooth['on']

        def record(phase, target, reached, points):
            # sim-to-real record: where each move (or smooth path) was sent and where the robot really stopped
            st.info.setdefault('real_stops', []).append({
                'phase': phase, 'points': points, 'target_deg': [round(float(v), 3) for v in target],
                'actual_deg': [round(float(v), 3) for v in reached],
                'max_err_deg': round(float(np.max(np.abs(reached - target))), 3), 't_s': round(time.time() - t_begin, 2)})

        for w in (smooth_paths(wps) if smooth['on'] else wps):
            if w['kind'] == 'wait':
                t_end = time.time() + w['seconds']
                shown = None
                while time.time() < t_end:
                    left = int(math.ceil(t_end - time.time()))
                    if left != shown:
                        sim.phase(w['phase'], f"REAL robot: {w['label']}: {left} s left...")
                        shown = left
                    mirror(np.array(robot.joints()))
                    time.sleep(1.0 / POLL_HZ)
                if w['phase'] == 'wait':   # at the pick point: where the real robot really is
                    st.info['real_joints_at_pick_deg'] = [round(v, 3) for v in robot.joints()]
                continue
            if w['kind'] == 'grip' and w.get('squeeze') and pause_at_pick_s > 0:
                # TEMPORARY TEST (config/cell.json debug_pause_at_pick_s): hold at the pick location with the fingers
                # at the plan's approach opening (as in the Sim). Opening fully here would swing a finger that is
                # inside a cavity across it and change the grasp.
                t_end = time.time() + pause_at_pick_s
                shown = None
                while time.time() < t_end:
                    left = int(math.ceil(t_end - time.time()))
                    if left != shown:
                        sim.phase('pause', f'TEST PAUSE at the pick location (fingers at the approach opening): {left} s left...')
                        shown = left
                    mirror(np.array(robot.joints()))
                    time.sleep(1.0 / POLL_HZ)
            if w['kind'] == 'grip':
                sim.phase(w['phase'], f"REAL gripper: {w['label']}" if gripper is not None else
                          f"Gripper (sim only): {w['label']}")
                if gripper is not None:
                    if w.get('squeeze'):
                        pos = 0 if w.get('inside') else 255   # until the fingers stop on the part
                    else:
                        pos = width_to_pos(w['width'])
                    gst = _grip(gripper, pos, g_speed, g_force, robot, mirror, w['phase'])
                    if w.get('squeeze'):
                        held = gst['object'] in (1, 2)
                        closed_mm = pos_to_width(gst['position'])
                        st.info['real_closed_width_mm'] = round(closed_mm, 1)
                        grip_notes.append(('the gripper felt the part' if held else
                                           'the gripper did NOT feel a part (it went to its end stop)') +
                                          f', closed to {closed_mm:.1f} mm (sim plan {plan["widthMm"]:.1f} mm)')
                        st.info['real_grip_object'] = held
                else:
                    width['now'] = w['width']
                if carried['on'] and not w['attached']:
                    sim._set_part_qpos(np.array(plan['T_wp_placed']))
                carried['on'] = bool(w['attached'])
                mirror(np.array(robot.joints()))
                continue
            moves = w['moves'] if w['kind'] == 'path' else [w]
            if gripper is None and moves[-1]['width'] is not None:
                width['now'] = moves[-1]['width']
            if len(moves) > 1 and smooth['on']:
                # one smooth motion through all the points; it only stops at the last one
                k += len(moves)
                labels = ', then '.join(dict.fromkeys(mv['label'] for mv in moves))
                sim.phase(moves[-1]['phase'], f"REAL robot: {labels} (smooth, {len(moves)} points, {speed:.0f}% speed)")
                pts = [np.degrees(mv['q']) for mv in moves]
                blend = [NEAR_PART_BLEND_MM if mv['phase'] in NEAR_PART_PHASES else BLEND_MM for mv in moves]
                if robot.move_path(pts, speed, blend):
                    target = pts[-1]
                    reached = _wait_arrival(robot, target, mirror, moves[-1]['phase'])
                    record(moves[-1]['phase'], target, reached, len(moves))
                    continue
                smooth['on'] = False      # this controller has no path motion: one move per point, as before
                st.info['real_smooth'] = False
                k -= len(moves)
            for mv in moves:
                k += 1
                target = np.degrees(mv['q'])
                sim.phase(mv['phase'], f"REAL robot: {mv['label']} (move {k}/{n_moves}, {speed:.0f}% speed)")
                reply = robot.move_joints(target, speed)
                if reply is False:
                    raise PlanError(f"The controller refused the move to '{mv['phase']}' (moveByJoint returned False).")
                reached = _wait_arrival(robot, target, mirror, mv['phase'])
                record(mv['phase'], target, reached, 1)
        st.info['real_seconds'] = round(time.time() - t_begin, 1)
        moving = False
        if go_home:
            return (f"The robot is at the home pose ({n_moves} moves{', FAKE robot' if fake else ''}). "
                    'Real Sim can start now.')
        grip_text = ('Real gripper: ' + '; '.join(grip_notes) + '.') if gripper is not None and grip_notes else \
            ('The gripper was not driven (shown in the sim only).' if gripper is None else '')
        if plan.get('touch'):
            return (f"Real touch test done: the fingertips went to the marked point "
                    f"({plan['touch']['standoffMm']:g} mm before it), held, and came back home ({n_moves} moves, "
                    f"{time.time() - t_begin:.0f} s{', FAKE robot' if fake else ''}). Check on the part where they stopped.")
        if grasp_test:
            return (f"Real grasp test done: pick point, wait, close, hold, open, home ({n_moves} moves, "
                    f"{time.time() - t_begin:.0f} s{', FAKE robot' if fake else ''}). {grip_text}")
        return (f"Real robot finished the pick & place motion ({n_moves} moves, "
                f"{time.time() - t_begin:.0f} s{', FAKE robot' if fake else ''}). {grip_text}")
    except (Stopped, KeyboardInterrupt):
        if moving:
            robot.stop()
        raise PlanError(f"{stop_requested['why'] or 'Stopped.'} A stop command was sent to the robot.")
    except PlanError:
        if moving:
            robot.stop()
        raise
    except Exception as e:  # noqa: BLE001
        if moving:
            robot.stop()
        raise PlanError(f'Real robot error: {type(e).__name__}: {e}. A stop command was sent.') from e
    finally:
        signal.signal(signal.SIGTERM, old_term)
        robot.close()
        if gripper is not None:
            gripper.close()


def _grip(gripper, pos, speed, force, robot, mirror, name, timeout=8.0):
    """Gripper to pos (0 open .. 255 closed), the sim fingers following; returns the final status.
    The arm holds still (the robot is not commanded while the gripper moves)."""
    gripper.command(pos, speed, force)
    t0 = time.time()
    time.sleep(0.1)
    q = np.array(robot.joints())
    while True:
        ok, st = gripper.done(pos)
        mirror(q)
        if ok:
            return st
        if time.time() - t0 > timeout:
            raise PlanError(f"The gripper did not finish '{name}' within {timeout:.0f} s (position {st['position']}/255).")
        time.sleep(1.0 / POLL_HZ)


def _wait_arrival(robot, target, mirror, name):
    """Waits until the move has started, then until it has stopped at the target. Returns the joints reached."""
    t_start, started, last, still_since = time.time(), False, None, None
    while True:
        tick = time.time()
        now = np.array(robot.joints())
        mirror(now)
        state = robot.state()
        if state in ('e-stop', 'alarm', 'collision'):
            raise PlanError(f"The robot reported '{state}' during the move to '{name}'.")
        at_target = np.max(np.abs(now - target)) < 0.05
        moving = last is not None and np.max(np.abs(now - last)) > 0.01
        started = started or state == 'running' or moving or at_target
        if not started and time.time() - t_start > 5.0:
            raise PlanError(f"The robot did not start moving to '{name}' within 5 s (state '{state}').")
        if started and state != 'running':
            if at_target:
                break
            still_since = still_since or time.time()
            if time.time() - still_since > 1.0:
                break
        else:
            still_since = None
        last = now
        time.sleep(max(0.0, 1.0 / POLL_HZ - (time.time() - tick)))
    reached = np.array(robot.joints())
    err = float(np.max(np.abs(reached - target)))
    if err > ARRIVE_TOLERANCE_DEG:
        raise PlanError(f"The robot stopped {err:.1f} deg away from '{name}'; not continuing.")
    return reached
