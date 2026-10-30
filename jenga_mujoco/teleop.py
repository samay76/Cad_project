#!/usr/bin/env python3
"""
teleop.py - Robot Teleoperation & Episode Recorder (Joystick + Keyboard)

Move the robot arm using:
- Joystick: Left Stick (X/Y plane), Right Stick (Height), Hold LB (Slow mode)
- Keyboard: Arrow keys, U/J, etc.

Recording controls (Works on BOTH Controller and Keyboard):
- START button  or  'P' key     --> START Recording (🔴 REC)
- 'Y' button    or  ENTER key   --> SAVE Episode (Auto-syncs LeRobot dataset)
- 'X' button    or  'X' key     --> DISCARD Episode (Cancel and clear buffer)
- 'A' button    or  SPACE / 'G' --> Toggle Gripper (✊ / ✋)
- SELECT button or  'H' key     --> Return to Home Pose
- L3 button     or  'B' key     --> Reset Jenga Block
"""

import os
import sys

# Auto-detect and switch to ~/lerobot_env if executed with system Python
_VENV_PY = os.path.expanduser("~/lerobot_env/bin/python")
if "lerobot_env" not in sys.prefix and os.path.exists(_VENV_PY) and sys.argv and sys.argv[0] != "-c":
    try:
        import mujoco, evdev
    except ImportError:
        os.execv(_VENV_PY, [_VENV_PY] + sys.argv)

import time
import select
import termios
import tty
import threading
import queue
import numpy as np
import mujoco
import mujoco.viewer
from mujoco.viewer import glfw

try:
    import evdev
except ImportError:
    evdev = None

from build_robot import build
from world import reset_jenga_block, JENGA_INITIAL_POS, TARGET_BOX_POS, is_block_in_box
from recorder import solve_ik, DOWNWARD_QUAT, HOME_QPOS, EpisodeRecorder

GRIPPER_MAX = 255.0
CONTROL_FPS = 30

# Workspace limits for gripper_mount TCP to protect the robot and table
TCP_X_MIN, TCP_X_MAX = 0.20, 0.75
TCP_Y_MIN, TCP_Y_MAX = -0.40, 0.40
TCP_Z_MIN, TCP_Z_MAX = 0.165, 0.55

# Speed settings (meters per second)
NORMAL_SPEED_XY = 0.18
NORMAL_SPEED_Z = 0.14
SLOW_SPEED_XY = 0.05
SLOW_SPEED_Z = 0.04
STICK_DEADZONE = 0.12
KEY_STEP_XYZ = 0.015


def find_gamepad():
    """Scans system input devices to find a connected gamepad/controller."""
    if evdev is None:
        return None
    try:
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        keywords = ["pad", "controller", "joystick", "xbox", "shanwan", "zebronics", "gamepad"]
        for dev in devices:
            name_lower = dev.name.lower()
            if any(k in name_lower for k in keywords):
                return dev
        if os.path.exists("/dev/input/event20"):
            return evdev.InputDevice("/dev/input/event20")
    except Exception:
        pass
    return None


class GamepadReader(threading.Thread):
    """Background thread reading evdev gamepad events."""

    def __init__(self, device, command_queue):
        super().__init__(daemon=True)
        self.device = device
        self.cmd_queue = command_queue
        self.running = True
        self.axes = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "rt": 0.0}
        self.buttons = {k: False for k in ["a", "b", "x", "y", "lb", "rb", "start", "select", "l3", "r3", "mode"]}

    def run(self):
        try:
            for event in self.device.read_loop():
                if not self.running:
                    break
                if event.type == evdev.ecodes.EV_ABS:
                    code, val = event.code, event.value
                    if code == evdev.ecodes.ABS_X:
                        self.axes["lx"] = self._norm(val)
                    elif code == evdev.ecodes.ABS_Y:
                        self.axes["ly"] = -self._norm(val)
                    elif code == evdev.ecodes.ABS_RX:
                        self.axes["rx"] = self._norm(val)
                    elif code == evdev.ecodes.ABS_RY:
                        self.axes["ry"] = -self._norm(val)
                    elif code == evdev.ecodes.ABS_RZ:
                        self.axes["rt"] = val / 255.0

                elif event.type == evdev.ecodes.EV_KEY:
                    btn_map = {
                        evdev.ecodes.BTN_SOUTH: "GRIP",
                        evdev.ecodes.BTN_A: "GRIP",
                        evdev.ecodes.BTN_EAST: "OPEN",
                        evdev.ecodes.BTN_B: "OPEN",
                        evdev.ecodes.BTN_NORTH: "DISCARD",
                        evdev.ecodes.BTN_X: "DISCARD",
                        evdev.ecodes.BTN_WEST: "SAVE",
                        evdev.ecodes.BTN_Y: "SAVE",
                        evdev.ecodes.BTN_START: "START_REC",
                        evdev.ecodes.BTN_SELECT: "HOME",
                        evdev.ecodes.BTN_THUMBL: "RESET_BLOCK",
                        evdev.ecodes.BTN_THUMBR: "RANDOM_BLOCK",
                        evdev.ecodes.BTN_MODE: "EXIT",
                    }
                    if event.code in btn_map and event.value == 1:
                        self.cmd_queue.put(btn_map[event.code])

                    # Track bumpers for slow mode
                    if event.code in (evdev.ecodes.BTN_TL,):
                        self.buttons["lb"] = (event.value == 1)
        except Exception:
            pass

    def _norm(self, raw):
        norm = max(-1.0, min(1.0, raw / 32767.0))
        if abs(norm) < STICK_DEADZONE:
            return 0.0
        sign = 1.0 if norm > 0 else -1.0
        return sign * (abs(norm) - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)

    def stop(self):
        self.running = False


def terminal_listener(command_queue, is_running):
    """Background listener for keyboard input in the terminal."""
    if not sys.stdin.isatty():
        return
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while is_running():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                ch = sys.stdin.read(1)
                # Map keys
                if ch in ("p", "P"):
                    command_queue.put("START_REC")
                elif ch in ("\r", "\n", "y", "Y"):
                    command_queue.put("SAVE")
                elif ch in ("x", "X"):
                    command_queue.put("DISCARD")
                elif ch in (" ", "g", "G"):
                    command_queue.put("GRIP")
                elif ch in ("o", "O"):
                    command_queue.put("OPEN")
                elif ch in ("h", "H"):
                    command_queue.put("HOME")
                elif ch in ("b", "B"):
                    command_queue.put("RESET_BLOCK")
                elif ch in ("r", "R"):
                    command_queue.put("RANDOM_BLOCK")
                elif ch in ("q", "Q"):
                    command_queue.put("EXIT")
    except Exception:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class UnifiedController:
    """Robot controller handling simulation, IK, recording, gamepad, and keyboard."""

    def __init__(self, gamepad_device=None):
        self.model = build()
        self.data = mujoco.MjData(self.model)
        self.ik_data = mujoco.MjData(self.model)

        self.data.qpos[:6] = HOME_QPOS.copy()
        self.data.ctrl[:6] = HOME_QPOS.copy()
        self.data.ctrl[6] = 0.0
        mujoco.mj_forward(self.model, self.data)

        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "gripper_mount")
        self.target_tcp_pos = self.data.site_xpos[self.site_id].copy()
        self.target_qpos = HOME_QPOS.copy()
        self.gripper_norm = 0.0

        self.cmd_queue = queue.Queue()

        self.gamepad = None
        if gamepad_device is not None:
            self.gamepad = GamepadReader(gamepad_device, self.cmd_queue)
            self.gamepad.start()

        self.recorder = None
        self.recording = False
        self.running = True

    def post_command(self, cmd):
        self.cmd_queue.put(cmd)

    def update(self, dt):
        # 1. Process all queued commands (from controller, terminal, or 3D window)
        while not self.cmd_queue.empty():
            cmd = self.cmd_queue.get_nowait()
            self._execute_command(cmd)

        # 2. Process Gamepad analog joysticks if connected
        if self.gamepad is not None:
            slow_mode = self.gamepad.buttons.get("lb", False)
            speed_xy = SLOW_SPEED_XY if slow_mode else NORMAL_SPEED_XY
            speed_z = SLOW_SPEED_Z if slow_mode else NORMAL_SPEED_Z

            vx = self.gamepad.axes["ly"] * speed_xy
            vy = self.gamepad.axes["lx"] * speed_xy
            vz = self.gamepad.axes["ry"] * speed_z

            if abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(vz) > 1e-4:
                self.target_tcp_pos[0] += vx * dt
                self.target_tcp_pos[1] += vy * dt
                self.target_tcp_pos[2] += vz * dt
                self._clamp_and_solve_ik()

            rt_val = self.gamepad.axes.get("rt", 0.0)
            if rt_val > 0.15:
                self.gripper_norm = float(np.clip(rt_val * 0.86, 0.0, 0.86))

    def _execute_command(self, cmd):
        # Keyboard discrete movements
        if cmd == "KEY_UP":
            self.target_tcp_pos[0] += KEY_STEP_XYZ
            self._clamp_and_solve_ik()
        elif cmd == "KEY_DOWN":
            self.target_tcp_pos[0] -= KEY_STEP_XYZ
            self._clamp_and_solve_ik()
        elif cmd == "KEY_LEFT":
            self.target_tcp_pos[1] -= KEY_STEP_XYZ
            self._clamp_and_solve_ik()
        elif cmd == "KEY_RIGHT":
            self.target_tcp_pos[1] += KEY_STEP_XYZ
            self._clamp_and_solve_ik()
        elif cmd == "KEY_PGUP":
            self.target_tcp_pos[2] += KEY_STEP_XYZ
            self._clamp_and_solve_ik()
        elif cmd == "KEY_PGDN":
            self.target_tcp_pos[2] -= KEY_STEP_XYZ
            self._clamp_and_solve_ik()

        # Gripper
        elif cmd == "GRIP":
            self.gripper_norm = 0.0 if self.gripper_norm > 0.4 else 0.86
            print(f"\n[Gripper: {'CLOSED ✊' if self.gripper_norm > 0.4 else 'OPEN ✋'}]", flush=True)
        elif cmd == "OPEN":
            self.gripper_norm = 0.0
            print("\n[Gripper: OPEN ✋]", flush=True)

        # Recording
        elif cmd == "START_REC":
            self._toggle_recording()
        elif cmd == "SAVE":
            self._save_recording()
        elif cmd == "DISCARD":
            self._discard_recording()

        # Utilities
        elif cmd == "HOME":
            self.target_qpos = HOME_QPOS.copy()
            self.gripper_norm = 0.0
            self.ik_data.qpos[:6] = self.target_qpos.copy()
            mujoco.mj_forward(self.model, self.ik_data)
            self.target_tcp_pos = self.ik_data.site_xpos[self.site_id].copy()
            print("\n[Arm returned to HOME posture]", flush=True)

        elif cmd == "RESET_BLOCK":
            reset_jenga_block(self.model, self.data, pos=JENGA_INITIAL_POS)
            print("\n[Jenga block reset to standard position]", flush=True)

        elif cmd == "RANDOM_BLOCK":
            jx = np.random.uniform(-0.025, 0.025)
            jy = np.random.uniform(-0.025, 0.025)
            yaw = np.random.uniform(-np.pi / 12, np.pi / 12)
            quat = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
            bpos = np.array([JENGA_INITIAL_POS[0] + jx, JENGA_INITIAL_POS[1] + jy, JENGA_INITIAL_POS[2]])
            reset_jenga_block(self.model, self.data, pos=bpos, quat=quat)
            print(f"\n[Jenga block randomized at x={bpos[0]:.3f}, y={bpos[1]:.3f}]", flush=True)

        elif cmd == "EXIT":
            self.running = False

    def _clamp_and_solve_ik(self):
        self.target_tcp_pos[0] = np.clip(self.target_tcp_pos[0], TCP_X_MIN, TCP_X_MAX)
        self.target_tcp_pos[1] = np.clip(self.target_tcp_pos[1], TCP_Y_MIN, TCP_Y_MAX)
        self.target_tcp_pos[2] = np.clip(self.target_tcp_pos[2], TCP_Z_MIN, TCP_Z_MAX)

        self.ik_data.qpos[:6] = self.data.qpos[:6].copy()
        ok, sol = solve_ik(self.model, self.ik_data, "gripper_mount", self.target_tcp_pos, DOWNWARD_QUAT)
        if ok:
            self.target_qpos = sol.copy()

    def _toggle_recording(self):
        if not self.recording:
            if self.recorder is None:
                self.recorder = EpisodeRecorder(output_dir="data/episodes", fps=20, save_videos=True)
            self.recorder.start_episode(instruction="CS66 teleoperated Jenga pick-and-place")
            self.recording = True
            print("\n" + "=" * 65, flush=True)
            print("🔴 RECORDING STARTED! Move the robot to pick and place.", flush=True)
            print("   👉 Press 'Y' (or ENTER) at any time to SAVE the episode", flush=True)
            print("   👉 Press 'X' at any time to DISCARD and try again", flush=True)
            print("=" * 65 + "\n", flush=True)
        else:
            self.recording = False
            num_steps = len(self.recorder.states) if self.recorder else 0
            print("\n" + "-" * 65, flush=True)
            print(f"⏹️ Recording paused ({num_steps} steps).", flush=True)
            print("   👉 Press 'Y' (or ENTER) to SAVE", flush=True)
            print("   👉 Press 'X' to DISCARD", flush=True)
            print("-" * 65 + "\n", flush=True)

    def _save_recording(self):
        if self.recorder is not None and len(self.recorder.states) > 0:
            self.recording = False
            num_steps = len(self.recorder.states)
            print("\n" + "=" * 65, flush=True)
            print(f"💾 SAVING EPISODE ({num_steps} steps)...", flush=True)
            print("   Rendering camera views and updating LeRobot dataset...", flush=True)
            print("   Please wait ~3 seconds...", flush=True)
            print("=" * 65, flush=True)

            in_box = is_block_in_box(self.model, self.data, target_box_pos=TARGET_BOX_POS)
            path = self.recorder.save_episode(model=self.model, success=in_box)

            print("\n" + "=" * 65, flush=True)
            print(f"✓ EPISODE SAVED: {os.path.basename(path)}", flush=True)
            print(f"✓ Block inside target box: {'YES (Success!)' if in_box else 'NO (Missed)'}", flush=True)
            print("✓ LeRobot dataset (data/lerobot_jenga/) automatically updated!", flush=True)
            print("   👉 Ready! Press START (or 'P') to record another episode.", flush=True)
            print("=" * 65 + "\n", flush=True)
            self.recorder.reset_buffers()
        else:
            print("\n⚠️ Nothing to save! Press START (or 'P') first to record an episode.\n", flush=True)

    def _discard_recording(self):
        if self.recorder is not None and len(self.recorder.states) > 0:
            self.recording = False
            self.recorder.reset_buffers()
            print("\n✗ Episode DISCARDED (Memory cleared). Ready for new try.\n", flush=True)


def main():
    pad_device = find_gamepad()

    print("\n" + "=" * 65)
    print(" ROBOT TELEOPERATION & RECORDER (JOYSTICK + KEYBOARD)")
    print("=" * 65)
    if pad_device:
        print(f" ✓ Gamepad Connected : {pad_device.name}")
        print("   Sticks: Move Arm | LB: Slow mode | A: Gripper")
        print("   START: Record | Y: Save | X: Discard")
    else:
        print(" ℹ No Gamepad found: Using Keyboard Controls.")
    print(" ✓ Keyboard Controls Active:")
    print("   'P': Record | ENTER or 'Y': Save | 'X': Discard")
    print("   SPACE: Gripper | Arrow Keys: Move | 'H': Home | 'B': Reset Block")
    print("=" * 65 + "\n")

    cmd_queue = queue.Queue()
    controller = UnifiedController(pad_device)
    controller.cmd_queue = cmd_queue

    # Start terminal keyboard listener
    term_thread = threading.Thread(
        target=terminal_listener,
        args=(cmd_queue, lambda: controller.running),
        daemon=True,
    )
    term_thread.start()

    # GLFW keys for the 3D window
    glfw_map = {
        glfw.KEY_P: "START_REC",
        glfw.KEY_ENTER: "SAVE",
        glfw.KEY_Y: "SAVE",
        glfw.KEY_X: "DISCARD",
        glfw.KEY_BACKSPACE: "DISCARD",
        glfw.KEY_SPACE: "GRIP",
        glfw.KEY_G: "GRIP",
        glfw.KEY_O: "OPEN",
        glfw.KEY_H: "HOME",
        glfw.KEY_B: "RESET_BLOCK",
        glfw.KEY_R: "RANDOM_BLOCK",
        glfw.KEY_UP: "KEY_UP",
        glfw.KEY_DOWN: "KEY_DOWN",
        glfw.KEY_LEFT: "KEY_LEFT",
        glfw.KEY_RIGHT: "KEY_RIGHT",
        glfw.KEY_PAGE_UP: "KEY_PGUP",
        glfw.KEY_PAGE_DOWN: "KEY_PGDN",
        glfw.KEY_U: "KEY_PGUP",
        glfw.KEY_J: "KEY_PGDN",
        glfw.KEY_ESCAPE: "EXIT",
        glfw.KEY_Q: "EXIT",
    }

    def key_callback(keycode):
        if keycode in glfw_map:
            controller.post_command(glfw_map[keycode])

    with mujoco.viewer.launch_passive(controller.model, controller.data, key_callback=key_callback) as viewer:
        viewer.cam.lookat[:] = [0.35, 0.0, 0.2]
        viewer.cam.distance = 1.3
        viewer.cam.azimuth = 145
        viewer.cam.elevation = -25

        default_flags = viewer.opt.flags.copy()
        default_geomgroup = np.array([1, 1, 1, 1, 1, 1], dtype=np.uint8)

        dt = 1.0 / CONTROL_FPS
        rec_subsample = 0
        last_hud = 0

        while viewer.is_running() and controller.running:
            loop_start = time.time()

            controller.update(dt)

            alpha = 0.30
            controller.data.ctrl[:6] = (1.0 - alpha) * controller.data.ctrl[:6] + alpha * controller.target_qpos
            controller.data.ctrl[6] = controller.gripper_norm * GRIPPER_MAX

            substeps = int(dt / controller.model.opt.timestep)
            for _ in range(substeps):
                mujoco.mj_step(controller.model, controller.data)

            if controller.recording and controller.recorder is not None:
                rec_subsample += 1
                if rec_subsample % 3 in (0, 2):
                    act_vec = np.concatenate([controller.data.ctrl[:6], [controller.gripper_norm]])
                    controller.recorder.record_step_fast(controller.model, controller.data, act_vec)

            viewer.opt.flags[:] = default_flags
            viewer.opt.geomgroup[:] = default_geomgroup
            viewer.sync()

            # Dynamic live HUD in terminal
            if time.time() - last_hud > 0.25:
                last_hud = time.time()
                if controller.recording and controller.recorder is not None:
                    steps = len(controller.recorder.states)
                    sys.stdout.write(f"\r🔴 RECORDING ({steps:3d} steps) | Press 'Y' or ENTER to SAVE, 'X' to DISCARD   ")
                else:
                    sys.stdout.write(f"\r⚪ READY | Press 'P' (or START) to record | SPACE: Grip                       ")
                sys.stdout.flush()

            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    if controller.gamepad:
        controller.gamepad.stop()
    print("\n\nExited Teleoperation.")


if __name__ == "__main__":
    main()
