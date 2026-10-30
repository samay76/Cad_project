import os
import json
import time
import h5py
import cv2
import mujoco
import numpy as np
from cameras import CAMERA_NAMES, CAM_WRIST, CAM_OVERVIEW, CAM_LOW_ANGLE, render_camera
from world import reset_jenga_block, JENGA_INITIAL_POS, TARGET_BOX_POS

# Default home joint posture for the CS66 robot arm over the workspace
HOME_QPOS = np.array([3.14159, -1.2, 1.8, -2.2, -1.57, 0.0], dtype=np.float64)
DOWNWARD_QUAT = np.array([0.0, 0.707107, -0.707107, 0.0], dtype=np.float64)


def solve_ik(model, data, site_name, target_pos, target_quat, max_steps=250, tol=1e-4):
    """Solves numerical Inverse Kinematics for a given site to match target position and quaternion."""
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    damping = 1e-3

    target_mat = np.zeros(9)
    mujoco.mju_quat2Mat(target_mat, target_quat)
    target_mat = target_mat.reshape(3, 3)
    orig_qpos = data.qpos.copy()

    for _ in range(max_steps):
        mujoco.mj_forward(model, data)
        current_pos = data.site_xpos[site_id]
        err_pos = target_pos - current_pos

        current_mat = data.site_xmat[site_id].reshape(3, 3)
        err_rot = 0.5 * (np.cross(current_mat[:, 0], target_mat[:, 0]) +
                         np.cross(current_mat[:, 1], target_mat[:, 1]) +
                         np.cross(current_mat[:, 2], target_mat[:, 2]))
        err = np.concatenate([err_pos, err_rot])

        if np.linalg.norm(err_pos) < tol and np.linalg.norm(err_rot) < 1e-3:
            sol = data.qpos[:6].copy()
            data.qpos[:] = orig_qpos
            return True, sol

        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        J = np.vstack([jacp[:, :6], jacr[:, :6]])
        JJT = J @ J.T + damping * np.eye(6)
        dq = J.T @ np.linalg.solve(JJT, err)
        data.qpos[:6] += dq * 0.5

        for i in range(6):
            data.qpos[i] = np.clip(data.qpos[i], model.jnt_range[i, 0], model.jnt_range[i, 1])

    sol = data.qpos[:6].copy()
    data.qpos[:] = orig_qpos
    return False, sol


class EpisodeRecorder:
    """Records multi-camera observations, proprioceptive states, and actions

    into standard HDF5 format and MP4 video files.
    """

    def __init__(self, output_dir="data/episodes", fps=20, img_width=224, img_height=224, save_videos=True):
        self.output_dir = output_dir
        self.fps = fps
        self.img_width = img_width
        self.img_height = img_height
        self.save_videos = save_videos

        # The 3 RealSense perception cameras
        self.camera_names = [CAM_WRIST, CAM_OVERVIEW, CAM_LOW_ANGLE]

        os.makedirs(self.output_dir, exist_ok=True)
        if self.save_videos:
            os.makedirs(os.path.join(self.output_dir, "videos"), exist_ok=True)

        self.episode_index = self._get_next_episode_index()
        self.reset_buffers()

    def _get_next_episode_index(self):
        """Scans output_dir for existing episode files to determine the next index."""
        existing = [f for f in os.listdir(self.output_dir) if f.startswith("episode_") and f.endswith(".h5")]
        if not existing:
            return 0
        indices = [int(f.split("_")[1].split(".")[0]) for f in existing if f.split("_")[1].split(".")[0].isdigit()]
        return max(indices) + 1 if indices else 0

    def reset_buffers(self):
        """Clears memory buffers for a new recording episode."""
        self.images = {cam: [] for cam in self.camera_names}
        self.states = []       # Proprioception: 6 arm joints + 1 gripper state
        self.actions = []      # Commanded action: 6 arm joint targets + 1 gripper command
        self.tcp_poses = []    # 3D position [x,y,z] + orientation [w,x,y,z]
        self.qpos_history = [] # Snapshots for fast deferred rendering
        self.timestamps = []
        self.instruction = ""
        self.start_time = None

    def start_episode(self, instruction="Pick up the Jenga block and place it inside the box"):
        """Starts a fresh episode recording."""
        self.reset_buffers()
        self.instruction = instruction
        self.start_time = time.time()

    def record_step(self, model, data, commanded_action):
        """Captures a single synchronized timestep (images, states, actions)."""
        # 1. Render all 3 camera streams
        for cam_name in self.camera_names:
            img = render_camera(model, data, cam_name, width=self.img_width, height=self.img_height)
            self.images[cam_name].append(img)

        # 2. Extract proprioceptive state: 6 joint angles + normalized gripper state (0=open, 1=closed)
        joint_q = data.qpos[:6].copy()
        gripper_q = float(np.clip(data.qpos[6] / 0.8, 0.0, 1.0))
        state_vec = np.concatenate([joint_q, [gripper_q]], dtype=np.float32)
        self.states.append(state_vec)

        # 3. Commanded action: 6 joint angle commands + normalized gripper command (0=open, 1=closed)
        act_vec = np.array(commanded_action, dtype=np.float32)
        self.actions.append(act_vec)

        # 4. TCP Pose (Position + Quaternion of gripper_mount site)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_mount')
        tcp_pos = data.site_xpos[site_id].copy()
        tcp_quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(tcp_quat, data.site_xmat[site_id])
        tcp_pose = np.concatenate([tcp_pos, tcp_quat], dtype=np.float32)
        self.tcp_poses.append(tcp_pose)

        # 5. Timestamp
        t_elapsed = len(self.states) / float(self.fps)
        self.timestamps.append(t_elapsed)

    def record_step_fast(self, model, data, commanded_action):
        """Instant (<0.05ms) trajectory recording without blocking simulation on image rendering."""
        joint_q = data.qpos[:6].copy()
        gripper_q = float(np.clip(data.qpos[6] / 0.8, 0.0, 1.0))
        state_vec = np.concatenate([joint_q, [gripper_q]], dtype=np.float32)
        self.states.append(state_vec)

        act_vec = np.array(commanded_action, dtype=np.float32)
        self.actions.append(act_vec)

        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_mount')
        tcp_pos = data.site_xpos[site_id].copy()
        tcp_quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mat2Quat(tcp_quat, data.site_xmat[site_id])
        tcp_pose = np.concatenate([tcp_pos, tcp_quat], dtype=np.float32)
        self.tcp_poses.append(tcp_pose)

        self.qpos_history.append(data.qpos.copy())

        t_elapsed = len(self.states) / float(self.fps)
        self.timestamps.append(t_elapsed)

    def save_episode(self, success=True, model=None):
        """Saves recorded episode to HDF5 and optional MP4 video files."""
        num_steps = len(self.states)
        if num_steps == 0:
            print("Warning: Attempted to save empty episode. Skipping.")
            return None

        # Deferred rendering if images were not rendered per-step
        if len(self.images[self.camera_names[0]]) == 0 and len(self.qpos_history) > 0 and model is not None:
            print(f" Rendering {len(self.qpos_history)} demonstration frames across 3 cameras...")
            scratch_data = mujoco.MjData(model)
            for qpos in self.qpos_history:
                scratch_data.qpos[:] = qpos
                mujoco.mj_forward(model, scratch_data)
                for cam_name in self.camera_names:
                    img = render_camera(model, scratch_data, cam_name, width=self.img_width, height=self.img_height)
                    self.images[cam_name].append(img)

        h5_filename = f"episode_{self.episode_index:06d}.h5"
        h5_path = os.path.join(self.output_dir, h5_filename)

        # Convert lists to NumPy arrays
        states_arr = np.array(self.states, dtype=np.float32)
        actions_arr = np.array(self.actions, dtype=np.float32)
        tcp_arr = np.array(self.tcp_poses, dtype=np.float32)
        times_arr = np.array(self.timestamps, dtype=np.float32)

        # 1. Save standard HDF5 file
        with h5py.File(h5_path, 'w') as f:
            # Metadata attributes
            f.attrs['episode_index'] = self.episode_index
            f.attrs['num_steps'] = num_steps
            f.attrs['fps'] = self.fps
            f.attrs['duration_sec'] = float(times_arr[-1])
            f.attrs['success'] = bool(success)
            f.attrs['language_instruction'] = str(self.instruction)
            f.attrs['robot_type'] = "Elite_CS66_Robotiq_2F85"

            # Trajectory datasets
            f.create_dataset('action', data=actions_arr, compression='gzip')
            f.create_dataset('timestamp', data=times_arr, compression='gzip')

            # Observations group
            obs_grp = f.create_group('observation')
            obs_grp.create_dataset('state', data=states_arr, compression='gzip')
            obs_grp.create_dataset('tcp_pose', data=tcp_arr, compression='gzip')

            # Camera image datasets
            img_grp = obs_grp.create_group('images')
            for cam_name in self.camera_names:
                imgs_arr = np.array(self.images[cam_name], dtype=np.uint8)
                img_grp.create_dataset(cam_name, data=imgs_arr, compression='gzip')

        # 2. Save MP4 videos for visual inspection
        if self.save_videos:
            for cam_name in self.camera_names:
                vid_filename = f"episode_{self.episode_index:06d}_{cam_name}.mp4"
                vid_path = os.path.join(self.output_dir, "videos", vid_filename)
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(vid_path, fourcc, self.fps, (self.img_width, self.img_height))
                for frame_rgb in self.images[cam_name]:
                    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                    writer.write(frame_bgr)
                writer.release()

        # 3. Update dataset metadata index
        self._update_dataset_info(h5_filename, num_steps, success)

        # 4. Automatically sync to LeRobot dataset format
        try:
            sync_to_lerobot_dataset(self.output_dir, lerobot_dir="data/lerobot_jenga", fps=self.fps)
        except Exception as e:
            print(f"[Warning] Could not auto-sync to LeRobot dataset: {e}")

        print(f"✓ Saved Episode {self.episode_index:06d} -> {h5_path} ({num_steps} frames, success={success})")
        self.episode_index += 1
        return h5_path

    def _update_dataset_info(self, filename, num_steps, success):
        """Maintains a dataset_info.json file summarizing total episodes and frames."""
        info_path = os.path.join(self.output_dir, "dataset_info.json")
        info = {
            "dataset_name": "jenga_pick_and_place",
            "robot": "Elite CS66 + Robotiq 2F-85",
            "fps": self.fps,
            "image_resolution": [self.img_height, self.img_width],
            "action_dim": 7,
            "state_dim": 7,
            "cameras": self.camera_names,
            "total_episodes": 0,
            "total_frames": 0,
            "episodes": []
        }
        if os.path.exists(info_path):
            try:
                with open(info_path, 'r') as fp:
                    info = json.load(fp)
            except Exception:
                pass

        # Filter out existing entry with same filename to avoid duplicates
        existing_eps = [ep for ep in info.get("episodes", []) if ep.get("file") != filename]
        existing_eps.append({
            "file": filename,
            "steps": int(num_steps),
            "success": bool(success),
            "prompt": str(self.instruction)
        })
        existing_eps.sort(key=lambda ep: ep["file"])
        info["episodes"] = existing_eps
        info["total_episodes"] = len(info["episodes"])
        info["total_frames"] = sum(int(ep["steps"]) for ep in info["episodes"])

        with open(info_path, 'w') as fp:
            json.dump(info, fp, indent=2)


def sync_to_lerobot_dataset(episodes_dir="data/episodes", lerobot_dir="data/lerobot_jenga", fps=20):
    """Automatically converts all HDF5 episodes to the LeRobot dataset format."""
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError:
        try:
            from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
        except ImportError:
            return

    import glob
    import shutil

    episodes_dir = os.path.abspath(episodes_dir)
    lerobot_dir = os.path.abspath(lerobot_dir)
    h5_files = sorted(glob.glob(os.path.join(episodes_dir, "episode_*.h5")))
    if not h5_files:
        return

    cams = {
        'cam_wrist_d405': 'wrist',
        'cam_overview_d405': 'overview',
        'cam_low_angle_d435i': 'low'
    }
    img_meta = {'dtype': 'video', 'shape': (224, 224, 3), 'names': ['height', 'width', 'channel']}
    features = {f'observation.images.{v}': dict(img_meta) for v in cams.values()}
    features['observation.state'] = {'dtype': 'float32', 'shape': (7,), 'names': [f'q{i}' for i in range(6)] + ['gripper']}
    features['action'] = {'dtype': 'float32', 'shape': (7,), 'names': [f'a{i}' for i in range(6)] + ['gripper']}

    if os.path.exists(lerobot_dir):
        shutil.rmtree(lerobot_dir)

    ds = LeRobotDataset.create(
        repo_id='local/jenga_sim',
        fps=fps,
        features=features,
        root=lerobot_dir,
        robot_type='Elite_CS66_Robotiq_2F85',
        use_videos=True,
    )

    for path in h5_files:
        with h5py.File(path, 'r') as f:
            task = f.attrs.get('language_instruction', 'Pick up the Jenga block and place it inside the box')
            n = f['action'].shape[0]
            for t in range(n):
                frame = {f'observation.images.{v}': f[f'observation/images/{k}'][t] for k, v in cams.items()}
                frame['observation.state'] = f['observation/state'][t].astype(np.float32)
                frame['action'] = f['action'][t].astype(np.float32)
                frame['task'] = str(task)
                try:
                    ds.add_frame(frame)
                except TypeError:
                    ds.add_frame(frame, task=str(task))
        ds.save_episode()

    if hasattr(ds, 'finalize'):
        ds.finalize()
    print(f"✓ Automatically synchronized {len(h5_files)} episodes to LeRobot dataset: {lerobot_dir}")



def execute_expert_pick_and_place(model, data, recorder=None,
                                  viewer=None,
                                  target_box_pos=TARGET_BOX_POS,
                                  instruction="Pick up the Jenga block and place it inside the box",
                                  physics_steps_per_record=50,
                                  auto_save=True):
    """Executes a smooth pick-and-place demonstration using IK waypoints.

    If a recorder is provided, samples and logs synchronized observations at the target fps.
    If viewer is provided, synchronizes the 3D GUI window in real time.
    If auto_save is False, retains recorded data in memory until explicit recorder.save_episode().
    """
    if recorder is not None:
        recorder.start_episode(instruction=instruction)

    # 1. Home initialization
    data.qpos[:6] = HOME_QPOS
    data.ctrl[:6] = HOME_QPOS
    data.ctrl[6] = 0.0  # Open gripper
    mujoco.mj_forward(model, data)

    # Allow block to settle on table
    for _ in range(50):
        mujoco.mj_step(model, data)

    block_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'jenga_block')
    b_pos = data.xpos[block_id].copy()

    # Calculate keyframe waypoints
    # The 2F-85 finger pads dip ~14.3mm as they close. Grasp height at 0.174 clears the table while gripping block.
    ok1, q_pre = solve_ik(model, data, 'gripper_mount', np.array([b_pos[0], b_pos[1], 0.260]), DOWNWARD_QUAT)
    ok2, q_grasp = solve_ik(model, data, 'gripper_mount', np.array([b_pos[0], b_pos[1], 0.174]), DOWNWARD_QUAT)
    ok3, q_lift = solve_ik(model, data, 'gripper_mount', np.array([b_pos[0], b_pos[1], 0.270]), DOWNWARD_QUAT)
    ok4, q_box_pre = solve_ik(model, data, 'gripper_mount', np.array([target_box_pos[0], target_box_pos[1], 0.270]), DOWNWARD_QUAT)
    ok5, q_box_place = solve_ik(model, data, 'gripper_mount', np.array([target_box_pos[0], target_box_pos[1], 0.200]), DOWNWARD_QUAT)

    if not all([ok1, ok2, ok3, ok4, ok5]):
        print("Warning: IK solver failed on one or more waypoints.")
        return False

    def move_trajectory(target_q, target_grip_norm, duration_substeps):
        """Interpolates smoothly to target joint angles and gripper command."""
        start_q = data.ctrl[:6].copy()
        start_g = data.ctrl[6]
        target_g = target_grip_norm * 255.0  # Robotiq 2F-85 control range is 0 to 255

        for step in range(duration_substeps):
            alpha = (step + 1) / duration_substeps
            s = 0.5 * (1.0 - np.cos(alpha * np.pi))  # Cosine smoothstep
            cmd_q = (1.0 - s) * start_q + s * target_q
            cmd_g = (1.0 - s) * start_g + s * target_g

            data.ctrl[:6] = cmd_q
            data.ctrl[6] = cmd_g
            mujoco.mj_step(model, data)

            if viewer is not None and (step % 5 == 0):
                viewer.sync()
                time.sleep(model.opt.timestep * 5)

            # Record sample at the specified recording interval
            if recorder is not None and (step % physics_steps_per_record == 0):
                action_vec = np.concatenate([cmd_q, [target_grip_norm]])
                recorder.record_step(model, data, action_vec)

    # Execution Sequence:
    # 1. Approach above block
    move_trajectory(q_pre, 0.0, 300)
    # 2. Descend to block
    move_trajectory(q_grasp, 0.0, 300)
    # Settle
    for _ in range(80):
        mujoco.mj_step(model, data)
        if viewer is not None and (_ % 10 == 0):
            viewer.sync()
            time.sleep(model.opt.timestep * 10)
    # 3. Close gripper around block
    move_trajectory(q_grasp, 0.86, 250)
    for _ in range(120):
        mujoco.mj_step(model, data)
        if viewer is not None and (_ % 10 == 0):
            viewer.sync()
            time.sleep(model.opt.timestep * 10)
    # 4. Lift block up
    move_trajectory(q_lift, 0.86, 350)
    # 5. Transport to target box
    move_trajectory(q_box_pre, 0.86, 400)
    # 6. Lower into box
    move_trajectory(q_box_place, 0.86, 250)
    # 7. Release block
    move_trajectory(q_box_place, 0.0, 200)
    # 8. Retreat back to home pose
    move_trajectory(q_box_pre, 0.0, 250)
    move_trajectory(HOME_QPOS, 0.0, 350)

    # Final settle
    for _ in range(100):
        mujoco.mj_step(model, data)
        if viewer is not None and (_ % 10 == 0):
            viewer.sync()
            time.sleep(model.opt.timestep * 10)

    # Verify if block successfully landed inside the target box
    final_b_pos = data.xpos[block_id]
    in_box = bool(
        (target_box_pos[0] - 0.065 < final_b_pos[0] < target_box_pos[0] + 0.065) and
        (target_box_pos[1] - 0.050 < final_b_pos[1] < target_box_pos[1] + 0.050) and
        (final_b_pos[2] < 0.04)
    )

    if recorder is not None and auto_save:
        recorder.save_episode(success=in_box)

    return in_box
