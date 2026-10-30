#!/usr/bin/env python3
"""
eval_act.py - Closed-Loop Multi-Block Evaluation of LeRobot ACT Policy in MuJoCo

Features:
- Spawns multiple Jenga blocks on the table (--num-blocks, default: 3).
- Sequential pick-and-place loop: robot tries to pick blocks one by one.
- Self-recovering retry loop: if a grasp misses, smoothly returns to Home and retries
  until the block is picked or max attempts are reached.
- Supports live 3D viewer (--render) and multi-camera MP4 video recording (--save-video).
"""

import os
import sys

# Auto-detect and switch to ~/lerobot_env if executed with system Python
_LEROBOT_VENV_PY = os.path.expanduser("~/lerobot_env/bin/python")
if "lerobot_env" not in sys.prefix and os.path.exists(_LEROBOT_VENV_PY) and sys.argv and sys.argv[0] != "-c":
    try:
        import lerobot
    except ImportError:
        os.execv(_LEROBOT_VENV_PY, [_LEROBOT_VENV_PY] + sys.argv)

import time
import json
import argparse
import numpy as np
import cv2
import torch
import mujoco

# LeRobot imports (verified in LeRobot 0.4.4)
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.processor.pipeline import PolicyProcessorPipeline

# Simulation & Robot environment
from build_robot import build
from world import (
    reset_jenga_block,
    reset_all_blocks,
    is_block_in_box,
    count_blocks_in_box,
    JENGA_INITIAL_POS,
    TARGET_BOX_POS,
    DEFAULT_BLOCK_POSITIONS,
)
from cameras import CAM_WRIST, CAM_OVERVIEW, CAM_LOW_ANGLE, render_camera
from recorder import HOME_QPOS

GRIPPER_MAX_CTRL = 255.0
CONTROL_FPS = 20  # 20 Hz control loop matching training dataset


def parse_args():
    parser = argparse.ArgumentParser(
        description="Closed-loop evaluation of LeRobot ACT policy with multiple Jenga blocks."
    )
    parser.add_argument(
        "--ckpt",
        type=str,
        default="outputs/act_jenga/checkpoints/last/pretrained_model",
        help="Path to pretrained model checkpoint folder.",
    )
    parser.add_argument(
        "--num-blocks",
        type=int,
        default=3,
        help="Number of Jenga blocks placed on table (default: 3).",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=1,
        help="Number of evaluation episodes to run (default: 1).",
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=5,
        help="Maximum pick-and-place attempts / retries per episode (default: 5).",
    )
    parser.add_argument(
        "--steps-per-attempt",
        type=int,
        default=350,
        help="Control steps per pick attempt at 20 Hz (default: 350 = 17.5s).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for episode reproducibility (default: 42).",
    )
    parser.add_argument(
        "--randomize",
        action="store_true",
        help="Add small random position/yaw jitter to initial block placements.",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="Launch interactive 3D MuJoCo viewer window during evaluation.",
    )
    parser.add_argument(
        "--save-video",
        action="store_true",
        help="Save composite multi-camera MP4 video per episode in outputs/eval/.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="outputs/eval",
        help="Directory to save evaluation results and videos.",
    )
    return parser.parse_args()


def load_policy_and_processors(ckpt_dir, device):
    """Loads ACT policy and the pre/post-processing pipelines from checkpoint."""
    if not os.path.exists(ckpt_dir):
        raise FileNotFoundError(f"Checkpoint directory not found: {ckpt_dir}")

    print(f"\n[1/3] Loading ACT policy from: {ckpt_dir}")
    policy = ACTPolicy.from_pretrained(ckpt_dir)
    policy.to(device)
    policy.eval()

    print("[2/3] Loading policy observation preprocessor...")
    preprocessor = PolicyProcessorPipeline.from_pretrained(
        ckpt_dir, "policy_preprocessor.json"
    )

    print("[3/3] Loading policy action postprocessor...")
    postprocessor = PolicyProcessorPipeline.from_pretrained(
        ckpt_dir, "policy_postprocessor.json"
    )

    return policy, preprocessor, postprocessor


def smooth_return_home(model, data, duration_steps=40, substeps=25, viewer=None):
    """Smoothly interpolates the robot arm back to HOME_QPOS and opens gripper."""
    start_q = data.ctrl[:6].copy()
    target_q = HOME_QPOS.copy()
    data.ctrl[6] = 0.0  # Open gripper

    for step in range(duration_steps):
        alpha = (step + 1) / duration_steps
        s = 0.5 * (1.0 - np.cos(alpha * np.pi))
        data.ctrl[:6] = (1.0 - s) * start_q + s * target_q
        for _ in range(substeps):
            mujoco.mj_step(model, data)
        if viewer is not None:
            viewer.sync()


def run_multi_block_episode(
    ep_idx,
    model,
    data,
    policy,
    preprocessor,
    postprocessor,
    device,
    num_blocks=3,
    max_attempts=5,
    steps_per_attempt=350,
    randomize=False,
    use_viewer=False,
    viewer=None,
    video_writer=None,
):
    """Executes closed-loop evaluation picking multiple blocks one by one with retries."""
    sim_dt = model.opt.timestep
    substeps_per_step = int(1.0 / (CONTROL_FPS * sim_dt))

    # 1. Reset all blocks on the table
    reset_all_blocks(model, data, num_blocks=num_blocks, randomize=randomize)

    # 2. Reset arm to standard HOME pose
    data.qpos[:6] = HOME_QPOS.copy()
    data.ctrl[:6] = HOME_QPOS.copy()
    data.ctrl[6] = 0.0  # Open gripper
    mujoco.mj_forward(model, data)

    # Settle physics
    for _ in range(50):
        mujoco.mj_step(model, data)
    if use_viewer and viewer is not None:
        viewer.sync()

    remaining_blocks = list(range(num_blocks))
    placed_blocks = []
    total_steps = 0
    attempt = 0

    print(f"\n--- Episode {ep_idx}: {num_blocks} Jenga Blocks on Table ---")
    for i in range(num_blocks):
        name = "jenga_block" if i == 0 else f"jenga_block_{i}"
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        pos = data.xpos[bid]
        print(f"  Block {i}: pos = [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

    while attempt < max_attempts and len(remaining_blocks) > 0:
        attempt += 1
        print(f"\n[Attempt {attempt}/{max_attempts}] Targeting remaining blocks ({len(remaining_blocks)} left)...")

        # Reset ACT policy internal buffers so it plans fresh from home pose
        policy.reset()
        current_ctrl = data.ctrl[:6].copy()

        attempt_placed = False
        consecutive_in_box = 0

        for step in range(steps_per_attempt):
            total_steps += 1

            # A. Render 3 camera views (all remaining blocks are visible)
            img_wrist = render_camera(model, data, CAM_WRIST, width=224, height=224)
            img_overview = render_camera(model, data, CAM_OVERVIEW, width=224, height=224)
            img_low = render_camera(model, data, CAM_LOW_ANGLE, width=224, height=224)

            if video_writer is not None:
                composite = np.concatenate([img_overview, img_wrist, img_low], axis=1)
                video_writer.write(cv2.cvtColor(composite, cv2.COLOR_RGB2BGR))

            # B. Read state
            joint_q = data.qpos[:6].copy()
            gripper_q = float(np.clip(data.qpos[6] / 0.8, 0.0, 1.0))
            state_vec = np.concatenate([joint_q, [gripper_q]], dtype=np.float32)

            # C. Format observation
            obs = {
                "observation.images.wrist": torch.from_numpy(img_wrist).permute(2, 0, 1).float() / 255.0,
                "observation.images.overview": torch.from_numpy(img_overview).permute(2, 0, 1).float() / 255.0,
                "observation.images.low": torch.from_numpy(img_low).permute(2, 0, 1).float() / 255.0,
                "observation.state": torch.from_numpy(state_vec).float(),
            }

            # D. Policy forward pass
            processed_obs = preprocessor(obs)
            with torch.no_grad():
                action_tensor = policy.select_action(processed_obs)
            post_out = postprocessor({"action": action_tensor})
            raw_action = post_out["action"][0].cpu().numpy().astype(np.float64)

            # E. Safety limits
            if np.isnan(raw_action).any():
                print("  Warning: NaN action detected! Halting attempt.")
                break

            arm_delta = raw_action[:6] - current_ctrl[:6]
            arm_delta_capped = np.clip(arm_delta, -0.1, 0.1)
            safe_arm = current_ctrl[:6] + arm_delta_capped
            for j in range(6):
                safe_arm[j] = np.clip(safe_arm[j], model.jnt_range[j, 0], model.jnt_range[j, 1])

            safe_gripper_norm = float(np.clip(raw_action[6], 0.0, 1.0))

            # F. Write control & step simulation
            data.ctrl[:6] = safe_arm
            data.ctrl[6] = safe_gripper_norm * GRIPPER_MAX_CTRL
            current_ctrl[:6] = safe_arm

            for _ in range(substeps_per_step):
                mujoco.mj_step(model, data)

            if use_viewer and viewer is not None:
                viewer.sync()

            # G. Check if any remaining block has landed inside the target box
            newly_placed = [b for b in remaining_blocks if is_block_in_box(model, data, block_idx=b)]
            if newly_placed:
                consecutive_in_box += 1
                if consecutive_in_box >= 15:  # Held in box for 0.75s
                    for b in newly_placed:
                        placed_blocks.append(b)
                        remaining_blocks.remove(b)
                        print(f"  ✓ BLOCK {b} PLACED IN TARGET BOX! (Total placed: {len(placed_blocks)}/{num_blocks})")
                    attempt_placed = True
                    break
            else:
                consecutive_in_box = 0

        # After attempt: allow release and smoothly return to Home posture
        data.ctrl[6] = 0.0  # Open gripper
        for _ in range(substeps_per_step * 10):
            mujoco.mj_step(model, data)
        if use_viewer and viewer is not None:
            viewer.sync()

        smooth_return_home(model, data, duration_steps=40, substeps=substeps_per_step, viewer=viewer)

        if attempt_placed:
            if len(remaining_blocks) == 0:
                print("\n🎉 ALL Jenga blocks successfully picked and placed inside the box!")
                break
            else:
                print(f"  -> Moving to pick next block! ({len(remaining_blocks)} remaining on table)")
        else:
            print("  ⚠️ Grasp not completed in this attempt. Returning to Home to RETRY...")

    success = len(placed_blocks) == num_blocks
    partial_success = len(placed_blocks) > 0

    return {
        "episode": ep_idx,
        "success": bool(success),
        "partial_success": bool(partial_success),
        "blocks_placed": len(placed_blocks),
        "total_blocks": num_blocks,
        "attempts": attempt,
        "total_steps": total_steps,
    }


def main():
    args = parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    print("=" * 70)
    print(" MULTI-BLOCK AUTONOMOUS ACT EVALUATION IN MUJOCO")
    print("=" * 70)
    print(f" Checkpoint         : {args.ckpt}")
    print(f" Jenga Blocks       : {args.num_blocks} blocks on table")
    print(f" Max Attempts / Ep  : {args.max_attempts} pick attempts")
    print(f" Steps / Attempt    : {args.steps_per_attempt} ({args.steps_per_attempt / CONTROL_FPS:.1f}s)")
    print(f" Randomize Jitter   : {args.randomize}")
    print(f" Device             : {device}")
    print(f" Interactive Viewer : {'ON (3D Window)' if args.render else 'OFF (Headless)'}")
    print(f" Save Video         : {'ON' if args.save_video else 'OFF'}")
    print(f" Output Directory   : {os.path.abspath(args.output_dir)}")
    print("=" * 70)

    os.makedirs(args.output_dir, exist_ok=True)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # 1. Load Policy and Processors
    policy, preprocessor, postprocessor = load_policy_and_processors(args.ckpt, device)

    # 2. Build Simulation Environment with Multi-Blocks
    print(f"\nCompiling MuJoCo simulation model with {args.num_blocks} Jenga blocks...")
    model = build(num_blocks=args.num_blocks)
    data = mujoco.MjData(model)

    episode_results = []
    viewer = None

    if args.render:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.lookat[:] = [0.45, 0.0, 0.15]
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 145
        viewer.cam.elevation = -25

    try:
        for ep in range(1, args.episodes + 1):
            video_writer = None
            if args.save_video:
                vid_path = os.path.join(args.output_dir, f"eval_ep{ep:03d}.mp4")
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                video_writer = cv2.VideoWriter(vid_path, fourcc, CONTROL_FPS, (224 * 3, 224))

            res = run_multi_block_episode(
                ep_idx=ep,
                model=model,
                data=data,
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                device=device,
                num_blocks=args.num_blocks,
                max_attempts=args.max_attempts,
                steps_per_attempt=args.steps_per_attempt,
                randomize=args.randomize,
                use_viewer=args.render,
                viewer=viewer,
                video_writer=video_writer,
            )

            if video_writer is not None:
                video_writer.release()

            episode_results.append(res)
            res_status = "ALL BLOCKS PLACED ✓" if res["success"] else f"PARTIAL ({res['blocks_placed']}/{res['total_blocks']})"
            print(
                f"\n>>> Episode {ep} Result: {res_status} | "
                f"Attempts: {res['attempts']} | Total Steps: {res['total_steps']}"
            )

    finally:
        if viewer is not None:
            viewer.close()

    # Summary table
    print("\n" + "=" * 70)
    print(" EVALUATION SUMMARY")
    print("=" * 70)
    for r in episode_results:
        print(f"Ep {r['episode']}: Placed {r['blocks_placed']}/{r['total_blocks']} blocks in {r['attempts']} attempts.")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
