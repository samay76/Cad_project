#!/usr/bin/env python3
"""
train_act.py - Train / Fine-Tune LeRobot ACT Policy on Jenga Demonstrations

Launches the LeRobot ACT training pipeline using the converted dataset in data/lerobot_jenga.
Saves model checkpoints, pre/postprocessors, and loss logs to outputs/.
"""

import os
import sys

# Auto-detect and switch to ~/lerobot_env if executed with system Python
_VENV_PY = os.path.expanduser("~/lerobot_env/bin/python")
if "lerobot_env" not in sys.prefix and os.path.exists(_VENV_PY) and sys.argv and sys.argv[0] != "-c":
    try:
        import lerobot
    except ImportError:
        os.execv(_VENV_PY, [_VENV_PY] + sys.argv)

import argparse
import subprocess

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DATASET = os.path.join(BASE_DIR, "data", "lerobot_jenga")
DEFAULT_OUTPUT = os.path.join(BASE_DIR, "outputs", "act_jenga_20k")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Train an ACT policy on Jenga pick-and-place dataset."
    )
    parser.add_argument(
        "--dataset-root",
        type=str,
        default=DEFAULT_DATASET,
        help=f"Path to LeRobot dataset directory (default: {DEFAULT_DATASET}).",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=DEFAULT_OUTPUT,
        help=f"Directory to save trained model checkpoints (default: {DEFAULT_OUTPUT}).",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=20000,
        help="Total training gradient update steps (default: 20000).",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=8,
        help="Training batch size (default: 8 for GTX 1650 4GB).",
    )
    parser.add_argument(
        "--save-freq",
        type=int,
        default=5000,
        help="Frequency (steps) to save intermediate checkpoints (default: 5000).",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume training from existing checkpoint in output directory.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 70)
    print(" LEROBOT ACT POLICY TRAINING PIPELINE")
    print("=" * 70)
    print(f" Dataset Directory  : {os.path.abspath(args.dataset_root)}")
    print(f" Output Directory   : {os.path.abspath(args.output_dir)}")
    print(f" Training Steps     : {args.steps}")
    print(f" Batch Size         : {args.batch_size}")
    print(f" Save Frequency     : Every {args.save_freq} steps")
    print(f" Resume Training    : {args.resume}")
    print("=" * 70)

    # Verify dataset exists
    if not os.path.exists(args.dataset_root):
        print(f"\n❌ Error: Dataset directory not found: {args.dataset_root}")
        print("Please record at least 1 episode using `python teleop.py` first.")
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    # Locate lerobot-train binary inside virtualenv
    venv_bin = os.path.dirname(sys.executable)
    lerobot_train_bin = os.path.join(venv_bin, "lerobot-train")
    if not os.path.exists(lerobot_train_bin):
        lerobot_train_bin = "lerobot-train"

    cmd = [
        lerobot_train_bin,
        "--policy.type=act",
        "--dataset.repo_id=local/jenga_sim",
        f"--dataset.root={os.path.abspath(args.dataset_root)}",
        f"--batch_size={args.batch_size}",
        "--num_workers=1",
        f"--steps={args.steps}",
        f"--output_dir={os.path.abspath(args.output_dir)}",
        f"--save_freq={args.save_freq}",
        "--policy.push_to_hub=false",
    ]

    if args.resume:
        cmd.append("--resume=true")

    print("\nExecuting command:")
    print(" ".join(cmd) + "\n")

    try:
        proc = subprocess.run(cmd)
        if proc.returncode == 0:
            print("\n" + "=" * 70)
            print(" ✓ TRAINING COMPLETED SUCCESSFULLY!")
            print(f" Model saved to: {os.path.join(args.output_dir, 'checkpoints', 'last', 'pretrained_model')}")
            print("=" * 70)
            print("\nYou can now test your trained model with:")
            print(f" python eval_act.py --ckpt {os.path.join(args.output_dir, 'checkpoints', 'last', 'pretrained_model')} --render")
        else:
            print(f"\nTraining process exited with code {proc.returncode}")
            sys.exit(proc.returncode)
    except KeyboardInterrupt:
        print("\nTraining interrupted by user.")


if __name__ == "__main__":
    main()
