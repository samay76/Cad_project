"""
replay_episode.py - Replay and Verify Recorded Pick-and-Place Episodes

Loads any recorded episode from data/episodes/ (e.g. episode_000001.h5)
and replays its exact commanded actions in MuJoCo.

Supports:
1. Interactive 3D GUI window: watch the robot move live (--gui)
2. Off-screen video export: saves a side-by-side multi-camera video (--video)
"""

import os
import sys

# Auto-detect and switch to ~/lerobot_env if executed with system Python
_VENV_PY = os.path.expanduser("~/lerobot_env/bin/python")
if "lerobot_env" not in sys.prefix and os.path.exists(_VENV_PY) and sys.argv and sys.argv[0] != "-c":
    try:
        import mujoco
    except ImportError:
        os.execv(_VENV_PY, [_VENV_PY] + sys.argv)

import time
import argparse
import h5py
import cv2
import numpy as np
import mujoco
import mujoco.viewer

from build_robot import build
from cameras import CAM_WRIST, CAM_OVERVIEW, CAM_LOW_ANGLE, render_camera
from world import TARGET_BOX_POS, is_block_in_box
from recorder import HOME_QPOS

GRIPPER_CTRL_MAX = 255.0


def replay_episode(episode_id=1, data_dir="data/episodes", use_gui=False, save_video=False):
    h5_path = os.path.join(data_dir, f"episode_{episode_id:06d}.h5")
    if not os.path.exists(h5_path):
        raise FileNotFoundError(f"Episode file not found: {h5_path}")

    print("=" * 65)
    print(f" REPLAYING RECORDED EPISODE: {os.path.basename(h5_path)}")
    print("=" * 65)

    # 1. Read episode data from HDF5
    with h5py.File(h5_path, "r") as h5:
        actions = h5["action"][:]                    # (T, 7)
        states = h5["observation/state"][:]          # (T, 7)
        num_steps = len(actions)
        instruction = h5.attrs.get("language_instruction", "Pick and place Jenga block")
        success_recorded = h5.attrs.get("success", True)
        fps = h5.attrs.get("fps", 20)

    print(f" Task Instruction  : {instruction}")
    print(f" Episode Timesteps : {num_steps} steps ({num_steps / fps:.2f} seconds)")
    print(f" Recorded Success  : {success_recorded}")
    print(f" Replay Mode       : {'Interactive GUI' if use_gui else 'Simulation headless'}")
    print("=" * 65)

    # 2. Build MuJoCo Model
    model = build()
    data = mujoco.MjData(model)
    sim_dt = model.opt.timestep
    substeps_per_step = int(1.0 / (fps * sim_dt))

    # 3. Initialize Robot to initial state of the episode
    initial_qpos = states[0][:6]
    initial_grip = states[0][6]
    data.qpos[:6] = initial_qpos
    data.ctrl[:6] = initial_qpos
    data.ctrl[6] = initial_grip * GRIPPER_CTRL_MAX

    # Set Jenga block pose near standard start
    block_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "jenga_block")
    mujoco.mj_step(model, data)

    video_writer = None
    video_out_path = None
    if save_video:
        video_out_path = f"data/episodes/replay_episode_{episode_id:06d}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        # Multi-view composite: 3 cameras side-by-side (3 * 320 x 240)
        video_writer = cv2.VideoWriter(video_out_path, fourcc, fps, (320 * 3, 240))
        print(f" Saving composite replay video to: {video_out_path}")

    # 4. Execution loop
    if use_gui:
        print("\nOpening interactive MuJoCo Viewer window...")
        print("Tip: You can use your mouse to rotate (left drag), pan (right drag), and zoom (scroll).")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set camera perspective
            viewer.cam.lookat[:] = [0.35, 0.0, 0.2]
            viewer.cam.distance = 1.3
            viewer.cam.azimuth = 145
            viewer.cam.elevation = -25

            for t in range(num_steps):
                if not viewer.is_running():
                    break
                step_start = time.time()
                act = actions[t]
                data.ctrl[:6] = act[:6]
                data.ctrl[6] = float(np.clip(act[6], 0.0, 1.0)) * GRIPPER_CTRL_MAX

                for _ in range(substeps_per_step):
                    mujoco.mj_step(model, data)

                viewer.sync()
                # Maintain real-time playback
                elapsed = time.time() - step_start
                sleep_time = (1.0 / fps) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
    else:
        for t in range(num_steps):
            act = actions[t]
            data.ctrl[:6] = act[:6]
            data.ctrl[6] = float(np.clip(act[6], 0.0, 1.0)) * GRIPPER_CTRL_MAX

            for _ in range(substeps_per_step):
                mujoco.mj_step(model, data)

            if video_writer is not None:
                img_wrist = render_camera(model, data, CAM_WRIST, width=320, height=240)
                img_over = render_camera(model, data, CAM_OVERVIEW, width=320, height=240)
                img_low = render_camera(model, data, CAM_LOW_ANGLE, width=320, height=240)
                # Concatenate horizontally: [Overview | Wrist | Low Angle]
                composite = np.concatenate([img_over, img_wrist, img_low], axis=1)
                video_writer.write(cv2.cvtColor(composite, cv2.COLOR_RGB2BGR))

        if video_writer is not None:
            video_writer.release()
            print(f"✓ Video saved successfully: {video_out_path}")

    # Allow physics to settle so released block drops into target box
    for _ in range(substeps_per_step * 10):
        mujoco.mj_step(model, data)

    # Check final success
    final_in_box = is_block_in_box(model, data, target_box_pos=TARGET_BOX_POS)
    print("\n" + "=" * 65)
    print(" REPLAY RESULT")
    print(f" Block Placed in Box: {'SUCCESS ✓' if final_in_box else 'MISSED'}")
    print("=" * 65)


def main():
    parser = argparse.ArgumentParser(description="Replay a recorded pick-and-place episode.")
    parser.add_argument("--episode", type=int, default=1, help="Episode number to replay (e.g. 0, 1, 2).")
    parser.add_argument("--gui", action="store_true", help="Launch interactive 3D viewer window.")
    parser.add_argument("--video", action="store_true", help="Save composite MP4 video of the replay.")
    args = parser.parse_args()

    replay_episode(episode_id=args.episode, use_gui=args.gui, save_video=args.video)


if __name__ == "__main__":
    main()
