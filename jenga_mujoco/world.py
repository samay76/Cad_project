import mujoco
import numpy as np

G = mujoco.mjtGeom

# Default task coordinates
TARGET_BOX_POS = np.array([0.55, 0.15, 0.0])
JENGA_INITIAL_POS = np.array([0.45, -0.10, 0.01])


# Multi-block standard layout on table within robot arm reach:
DEFAULT_BLOCK_POSITIONS = [
    np.array([0.45, -0.10, 0.0075]),  # Block 0: Standard center
    np.array([0.45, -0.04, 0.0075]),  # Block 1: Shifted +Y
    np.array([0.45, -0.16, 0.0075]),  # Block 2: Shifted -Y
    np.array([0.38, -0.10, 0.0075]),  # Block 3: Shifted -X
    np.array([0.52, -0.10, 0.0075]),  # Block 4: Shifted +X
]

WOOD_COLORS = [
    [0.82, 0.70, 0.54, 1.0],  # Natural birch
    [0.85, 0.65, 0.45, 1.0],  # Warm oak
    [0.75, 0.58, 0.38, 1.0],  # Dark pine
    [0.88, 0.75, 0.58, 1.0],  # Light maple
    [0.80, 0.55, 0.35, 1.0],  # Cedar
]


def add_world(spec, add_box=True, add_block=True, num_blocks=1):
    """Floor, table, target placement box, dynamic Jenga blocks and lighting.
    
    Table top surface is at z=0.
    """
    w = spec.worldbody

    # 1. Floor & Table
    w.add_geom(name='floor', type=G.mjGEOM_PLANE, size=[3, 3, 0.05],
               pos=[0, 0, -0.75], rgba=[0.55, 0.55, 0.55, 1])

    w.add_geom(name='table_top', type=G.mjGEOM_BOX, size=[0.6, 0.4, 0.02],
               pos=[0.3, 0, -0.02], rgba=[0.75, 0.6, 0.45, 1],
               friction=[1.0, 0.005, 0.0001])

    legs = [(-0.25, -0.35), (-0.25, 0.35), (0.85, -0.35), (0.85, 0.35)]
    for i, (x, y) in enumerate(legs):
        w.add_geom(name=f'leg{i}', type=G.mjGEOM_BOX, size=[0.025, 0.025, 0.365],
                   pos=[x, y, -0.395], rgba=[0.4, 0.3, 0.2, 1])

    # 2. Taped boundary marking on table (yellow flat tape)
    w.add_geom(name='boundary', type=G.mjGEOM_BOX, size=[0.09, 0.07, 0.0005],
               pos=[0.55, 0.15, 0.0005], rgba=[1.0, 0.88, 0.15, 1],
               contype=0, conaffinity=0)

    # 3. Target Placement Box (receptacle tray with walls)
    if add_box:
        bx, by = TARGET_BOX_POS[0], TARGET_BOX_POS[1]
        bw, bl, bh = 0.065, 0.050, 0.003
        wall_th = 0.003
        wall_h = 0.012

        # Tray bottom base
        w.add_geom(name='box_bottom', type=G.mjGEOM_BOX, size=[bw, bl, bh],
                   pos=[bx, by, bh], rgba=[0.22, 0.26, 0.34, 1.0],
                   friction=[1.0, 0.005, 0.0001])
        # 4 perimeter walls
        w.add_geom(name='box_wall_left', type=G.mjGEOM_BOX, size=[wall_th, bl, wall_h],
                   pos=[bx - bw, by, wall_h], rgba=[0.28, 0.33, 0.42, 1.0])
        w.add_geom(name='box_wall_right', type=G.mjGEOM_BOX, size=[wall_th, bl, wall_h],
                   pos=[bx + bw, by, wall_h], rgba=[0.28, 0.33, 0.42, 1.0])
        w.add_geom(name='box_wall_front', type=G.mjGEOM_BOX, size=[bw, wall_th, wall_h],
                   pos=[bx, by - bl, wall_h], rgba=[0.28, 0.33, 0.42, 1.0])
        w.add_geom(name='box_wall_back', type=G.mjGEOM_BOX, size=[bw, wall_th, wall_h],
                   pos=[bx, by + bl, wall_h], rgba=[0.28, 0.33, 0.42, 1.0])

    # 4. Dynamic Jenga Block(s) (Standard wooden block: 75 x 25 x 15 mm)
    if add_block and num_blocks > 0:
        for i in range(num_blocks):
            b_name = 'jenga_block' if i == 0 else f'jenga_block_{i}'
            j_name = 'jenga_freejoint' if i == 0 else f'jenga_freejoint_{i}'
            g_name = 'jenga_geom' if i == 0 else f'jenga_geom_{i}'
            b_pos = DEFAULT_BLOCK_POSITIONS[i] if i < len(DEFAULT_BLOCK_POSITIONS) else (JENGA_INITIAL_POS + [0, i * 0.05, 0])
            color = WOOD_COLORS[i % len(WOOD_COLORS)]

            b = w.add_body(name=b_name, pos=b_pos)
            b.add_freejoint(name=j_name)
            b.add_geom(name=g_name, type=G.mjGEOM_BOX,
                       size=[0.0375, 0.0125, 0.0075],  # 75 x 25 x 15 mm
                       mass=0.015,                      # 15 grams
                       friction=[1.2, 0.005, 0.0001],
                       rgba=color)

    # 5. Scene Lighting
    w.add_light(pos=[0.3, 0, 2], dir=[0, 0, -1])


def reset_jenga_block(model, data, pos=None, quat=None, block_idx=0):
    """Resets the 6-DoF position and velocity of a specific dynamic Jenga block."""
    j_name = 'jenga_freejoint' if block_idx == 0 else f'jenga_freejoint_{block_idx}'
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
    if joint_id < 0:
        return
    qpos_addr = model.jnt_qposadr[joint_id]
    qvel_addr = model.jnt_dofadr[joint_id]

    if pos is None:
        pos = DEFAULT_BLOCK_POSITIONS[block_idx] if block_idx < len(DEFAULT_BLOCK_POSITIONS) else JENGA_INITIAL_POS
    if quat is None:
        quat = [1.0, 0.0, 0.0, 0.0]

    data.qpos[qpos_addr:qpos_addr + 3] = pos
    data.qpos[qpos_addr + 3:qpos_addr + 7] = quat
    data.qvel[qvel_addr:qvel_addr + 6] = 0.0


def reset_all_blocks(model, data, num_blocks=1, randomize=False):
    """Resets all dynamic Jenga blocks to their starting table locations."""
    for i in range(num_blocks):
        base_pos = DEFAULT_BLOCK_POSITIONS[i] if i < len(DEFAULT_BLOCK_POSITIONS) else JENGA_INITIAL_POS
        if randomize:
            jx = np.random.uniform(-0.015, 0.015)
            jy = np.random.uniform(-0.015, 0.015)
            yaw = np.random.uniform(-np.pi / 12, np.pi / 12)
            quat = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
            pos = [base_pos[0] + jx, base_pos[1] + jy, base_pos[2]]
        else:
            pos = base_pos
            quat = [1.0, 0.0, 0.0, 0.0]
        reset_jenga_block(model, data, pos=pos, quat=quat, block_idx=i)


def is_block_in_box(model, data, block_idx=0, target_box_pos=TARGET_BOX_POS):
    """Checks whether a specific Jenga block has landed inside the perimeter of the target box."""
    b_name = "jenga_block" if block_idx == 0 else f"jenga_block_{block_idx}"
    block_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b_name)
    if block_id < 0:
        return False
    b_pos = data.xpos[block_id]
    return bool(
        (target_box_pos[0] - 0.065 < b_pos[0] < target_box_pos[0] + 0.065) and
        (target_box_pos[1] - 0.050 < b_pos[1] < target_box_pos[1] + 0.050) and
        (b_pos[2] < 0.04)
    )


def count_blocks_in_box(model, data, num_blocks=1, target_box_pos=TARGET_BOX_POS):
    """Returns a list of block indices that are currently resting inside the target box."""
    in_box = []
    for i in range(num_blocks):
        if is_block_in_box(model, data, block_idx=i, target_box_pos=target_box_pos):
            in_box.append(i)
    return in_box