# Simulating a taught CAD grasp in MuJoCo

Teach a grasp in the web CAD tool, then press **▶ Simulate in MuJoCo**. A MuJoCo window
opens with the same STEP part on the table, shows your grasp as a green ghost gripper, and
runs the pick & place with the CS66 + Robotiq 2F-85.

## One-time setup

```bash
# Web app (Node 22)
conda create -n cadweb -c conda-forge nodejs=22
# Simulator (MuJoCo + STEP conversion + convex decomposition)
conda create -n mujoco_cad -c conda-forge python=3.11
~/miniforge3/envs/mujoco_cad/bin/pip install mujoco trimesh coacd cadquery-ocp scipy numpy pillow
```

The web bridge uses `~/miniforge3/envs/mujoco_cad/bin/python`; override with `MUJOCO_PYTHON=/path/to/python npm run dev`.

## Workflow

```bash
conda activate cadweb && cd ~/Cad_project && npm run dev     # http://localhost:3000
```

1. Load the STEP part, then **Load Gripper**.
2. Place the gripper (Snap to point / Auto-place / gizmo), close until contact, **Save Grasp Pose**.
3. In **Saved Grasps** press ▶ on a grasp (or **Simulate in MuJoCo** for the selected one).
4. A MuJoCo window opens. The browser shows the progress and the final result / error.

### Choosing the gripper

The header has a **2F-85 | Hand-E** selector. The choice is remembered, and each saved grasp stores the
gripper it was taught with (badge in Saved Grasps). Clicking a grasp switches to its gripper, and ▶
simulates it with that gripper.

| | Robotiq 2F-85 | Robotiq Hand-E |
|---|---|---|
| Web model | `public/gripper/` (PickNik URDF) | `public/grippers/robotiq_hande/` (from `robotiq_hande_description`, Apache-2.0) |
| MuJoCo model | Menagerie `robotiq_2f85/2f85.xml` | built in `hande.py` from the same meshes |
| Opening | 0–85 mm (4-bar fingers) | 0–50 mm (parallel fingers, 2 × 25 mm travel) |
| Default TCP | 145 mm from the flange | 146.5 mm (fingertip at 156.5 mm incl. 11 mm coupler) |
| Grip force (sim) | `grip_force_limit` | `grip_force_limit_hande_N` (default 60 N) |

### Recording a motion (e.g. inside grip of a hollow part)

A single saved pose is always approached in a straight line along the TCP axis, with the fingers fully
open and closed to grip. For anything else, record the motion:

1. In **Saved Grasps** press **Record gripper motion** (button turns red).
2. Move the gripper the way the robot should go: gizmo, step buttons, finger opening slider. For a hollow
   cylinder, close the fingers, move above the bore, go down into it, then **open** the fingers
   until they press against the inner wall.
3. Press **Stop & save recording**. The grasp is saved as `Recorded Grasp N` with a `REC · inside` /
   `REC · outside` badge.
4. Press ▶. MuJoCo starts at the first recorded pose, replays the path relative to the part with the
   recorded finger widths, grips, carries the part to the tray, releases (fingers back to the width they
   had before gripping) and leaves along the recorded path in reverse.

Inside grip of `circle_Cad.step` (bore Ø35.7 mm, 39 mm high), tested through the UI:
Flip Approach (180°) so the fingers point down → Close (0mm) → Record → step 5 mm, Z − until the
gripper position relative to the part reads Z ≈ 30 mm → move the opening slider up (e.g. to 40 mm) →
Stop & save → ▶. Z = 25 mm is too deep: the thick part of the finger hits the rim, and the sim says how many mm
higher to stop. The web contact badge shows "contact" inside the hole because it uses box shapes;
ignore it there.

Grip direction: what the fingers do at the end of the recording, after the gripper has stopped moving.
Opening = inside grip, closing = outside grip. If they don't move, the app asks.

The part lies on the table in the **same orientation it has in the web viewer**, then settles under
gravity; the grasp is applied relative to the part's settled pose.

### Inspection camera

A camera stands on a post at the table edge (`cam_inspection`). After lifting, the robot brings the part
to the inspection point in front of it and turns it about its own centre of mass, like a turntable:
front, turned 90/180/-90 deg, then tilted to show the bottom and the top. At each position a still image
is saved. Then the part goes to the tray as before.

In the web app a **camera panel** opens in the 3D view when you press ▶. It shows the live camera feed
during the run and the inspection images as thumbnails, each with the slip of the part in the
gripper at that moment. Click a thumbnail for the full-size image.

- Settings: `inspection` in `sim_config.json` (camera position, inspection point, field of view,
  views, turn speed). Views the arm can't reach are skipped and listed in the result.
- Turning too fast makes the part slip in the 2F-85's weak simulated grip (about 6 mm per 90 deg turn at
  2 s per turn, under 1 mm at 4 s); the default is 4 s per turn.
- Files per run in `runs/<id>/`: `feed.jpg` (live frame) and `inspect_01.png`… (640×480).
- The camera is rendered by a helper process (`camera_feed.py`), because the MuJoCo window and an
  offscreen renderer can't share one process here.

## Sequence and checks

settle → **preview** (green ghost = taught grasp with the fingers at the pre-grip width, TCP axes
X red / Y green / Z blue, yellow dots = taught path) → **check** → start (single pose: `preGraspDistance`
back along TCP −Z; recording: first recorded pose) → approach (straight line, or the recorded path) →
grip (force-limited: close for outside, open for inside, stops on contact) → lift → inspection (camera on
the table, several positions) → transfer → place in
tray → release → retreat (taught path backwards) → home.

Before the robot moves, every waypoint is checked; on failure the ghost turns **red**, the reason is
shown in the window and in the browser, and nothing is executed:

- `Grasp unreachable at '<waypoint>'` – IK misses the target (residual in mm/deg is reported)
- `'<segment>' needs an arm reconfiguration mid-move` – straight-line approach/lift not feasible
- `Collision at '<waypoint>': '<link>' hits 'table_top'` – arm/gripper would hit the table or tray
- `finger pad ... intersects the part` – part wider than the opening (single pose), or the recorded
  motion runs a finger into the part (collision checks use the recorded finger width at every pose)
- `The part was not picked up` – grasp did not hold during lift

The final report includes the **TCP-vs-taught-grasp error** (actual sim TCP relative to the actual
part, compared with the exported pose) and the **slip during transport**.

## Frames ("what I see is what the robot does")

All transforms live in `frames.py`. Short version:

| Frame | Web tool | MuJoCo |
|---|---|---|
| Part | STEP native frame, mm | body `cad_part`, same frame × 0.001 |
| TCP | `GripperModel.root`, +Z approach, X closing | site `g_tcp` |
| Gripper base | URDF `robotiq_85_base_link`, TCP at +Z `tcpOffset` | Menagerie `g_base`; URDF base = (0,0,10.8 mm) + 180° about Z |

`T_world_tcp = T_world_part(settled) · T_part_tcp(exported, mm→m, quaternion xyzw→wxyz)`; IK drives `g_tcp` there.

## Files

| File | Purpose |
|---|---|
| `step_to_mesh.py` | STEP → `visual.stl` + CoACD `hull_XX.stl` + `meta.json` (exact steel mass/inertia). Cached in `assets/cad_parts/<hash>/` |
| `cad_part.py` | Adds the part to the scene (visual mesh display-only, hulls for contact, friction/priority for the pads) |
| `frames.py` | All frame transforms (documented) |
| `ik.py` | Multi-seed DLS IK that reports its residual |
| `taught_path.py` | Saved pose or recording → TCP path, finger widths, grip direction (inside/outside) |
| `camera_feed.py` | Inspection camera rendering in a helper process (live `feed.jpg` + stills) |
| `hande.py` | Robotiq Hand-E MuJoCo model (meshes converted once into `assets/hande/`) |
| `finger_collision.py` | Finger meshes split into convex pieces (CAD scene only), see below |
| `cad_pick_place.py` | The simulator (preview, checks, execution, status reporting) |
| `sim_config.json` | Pick/place positions, lift height, density, grip force, speeds |
| `build_robot.py` | `build(obj='jenga' \| 'cad_part', ...)` – Jenga stays the default |
| `../vite-plugins/mujocoBridge.ts` | Dev-server endpoints `/api/mujoco/simulate` and `/api/mujoco/status` |

**Finger collision shapes.** MuJoCo collides each mesh as its convex hull, which made the closed
fingertips ~38 mm wide instead of ~16 mm (the hull fills in the thin tip). In the CAD scene the finger
`tongue` and `follower` meshes are replaced by 8 CoACD pieces each (cached in
`assets/gripper_hulls/`), so the fingertips fit where they fit in the web tool. Note the finger gets
thick (~38 mm closed) from ~8 mm behind the TCP towards the gripper: only the tip goes into a bore.

The web finger opening (mm between the pad faces) is mapped to the MuJoCo actuator by a calibration run
at start-up (`FingerMap` in `cad_pick_place.py`).

Each button press writes `runs/<timestamp>/` (part file, request, status, log).

## Testing each step by hand

```bash
cd ~/Cad_project/jenga_mujoco
PY=~/miniforge3/envs/mujoco_cad/bin/python

# 1. Mesh conversion – compare the printed size with the web Properties panel; --view shows hulls
$PY step_to_mesh.py ../skf_cam_roller_305703_c-2z_0.stp --view

# 2. Scene integration – part on the table (Jenga scene still: --object jenga)
$PY build_robot.py --object cad_part

# 3+4. Grasp import + pick & place from an exported JSON (Saved Grasps → Export JSON)
$PY cad_pick_place.py --step ../skf_cam_roller_305703_c-2z_0.stp --grasps ~/Downloads/grasps_skf_cam_roller_305703_c-2z_0_stp.json
#   --grasp-name "Grasp Pose 1"   pick a grasp by name
#   --headless --snapshots out/   no window, PNG per phase
# A recorded grasp (with "trajectory") in the same JSON is replayed the same way.
```
