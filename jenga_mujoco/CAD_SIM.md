# CAD pick & place in MuJoCo

Two ways to pick a STEP part with the selected robot arm (Elite CS66 or EC66) and gripper (Robotiq 2F-85 or Hand-E):

- **Automatic** (the main workflow): click a point on the part, and the grasp and motion are calculated.
- **Taught**: place the gripper yourself, save the grasp, and press **▶ Simulate in MuJoCo**.

## One-time setup

```bash
# Web app (Node 22)
conda create -n cadweb -c conda-forge nodejs=22
# Simulator (MuJoCo + STEP conversion + convex decomposition)
conda create -n mujoco_cad -c conda-forge python=3.11
~/miniforge3/envs/mujoco_cad/bin/pip install mujoco trimesh coacd cadquery-ocp scipy numpy pillow
```

The web bridge uses `~/miniforge3/envs/mujoco_cad/bin/python`; override with `MUJOCO_PYTHON=/path/to/python npm run dev`.

## Automatic pick & place

```bash
conda activate cadweb && cd ~/Cad_project && npm run dev     # http://localhost:3000
```

In the **Auto Pick & Place** panel (left sidebar):

1. **Load CAD**: choose a `.step` / `.stp` file.
2. Click the part where one finger should go, then where the other should go. Numbered markers show the points. A
   third click starts over; **Clear Points** removes them.
   When both are marked, a see-through shape appears between them, showing the direction the fingers move (arrows).
   It's red if the grip can't work, and the panel says why.
3. **Plan to Points**: up to three complete solutions are listed, best first. Click one to show it: the gripper
   moves to the points, and the table, tray and planned TCP path are drawn (blue = free space, green = near the
   part, orange = carrying, purple = showing it to the camera).
4. Preview before running anything:
   - **Preview Here** plays the gripper and part in the viewport.
   - **Preview Robot** plays the planned motion with the arm in a MuJoCo window, without physics.
5. **Execute Pick & Place** runs it with full physics, with the live camera feed and inspection images in a panel in the
   viewport, and reports how close each finger came to its point.

Nothing is recorded or replayed: the motion is calculated from the marked points each time.

### What the two points mean

The surfaces at the points decide the grip:

- **Facing away from each other** (two outer sides): outside grip. The fingers close on the points.
- **Facing each other** (two sides of a bore or slot): inside grip. The fingers enter closed and open against the points.
- **Same direction, or more than 53 deg from facing directly opposite**: refused, because the fingers would slide off.

The points don't have to line up. The fingers close along the direction the two surfaces face, and both points just have to land
on the pads (about 21 mm wide). On a curved wall a flat pad touches at its edges first, so the fingers can stop a few mm short of a point.

The gripper goes exactly where you marked or nowhere: if the points can't be reached, the app says why and does not grip
somewhere else. One point on the inner wall and one on the outer wall is allowed, since you chose it, and the app points it out.

The points can land anywhere on the gripping face, so the gripper can sit higher or lower to clear the table or fit a bore.
Inside a bore the 2F-85 presses with the metal behind the pad, so the reported distance there is about 3 mm rather than ~0.

### Single-point search (command line only)

Surface points are sampled, mostly near the click, and a ray from each finds the opposite surface. Pairs whose
normals oppose each other inside the friction cone and that fit the gripper become candidates. Rays into the
material give an outside grip, and rays across free space give an inside grip.
Each pair is tried with approach directions around the closing axis, from straight down to 65 deg off vertical
(up to level for an inside grip), with the
gripping face on the contacts, measured from the gripper model at that opening (`gripper_geometry.py`).

`auto_pick_place.py --click ... --normal ...` searches for a grasp near one point. The app no longer uses this. The score weighs distance to the click, how well the surfaces oppose, distance to the centre of mass, how much of
the pad touches, and how vertical the approach is. Inside grips rank below outside grips. If nothing works near
the click, the whole part is searched and the app says so.

### How a candidate is checked (`auto_pick_place.py`)

Best first, until three complete motions are found. This applies to grasps from marked points too. A candidate is dropped at the first thing that fails: reach (IK),
the straight approach, the fingers at the grip (only the pads may touch), the straight lift, the target
(tried at several yaws), opening the fingers there, the retreat, and the free-space moves.

Sequence: home → pre-grasp → grasp → close → lift → inspection → transfer → place → open → retreat → home. Free-space moves and the transfer use
RRT-Connect when a direct joint move would collide (`motion_planner.py`). Collisions are checked against
the same shapes the physics uses, with the carried part attached to the gripper (`collision_check.py`).

### Limits

- The whole STEP file is one rigid object: the click guides the grasp, but one solid of an assembly can't be picked alone.
- The target is `place_xy` in `sim_config.json`. The part must fit the tray (inner 124 x 94 mm) and is placed in the orientation it was
  grasped, turned about the vertical if needed.
- The part is released onto the table and settles first. If that tips it, the plan uses the settled pose and the viewport says so.
- Inspection views are set in `inspection` in `sim_config.json` (`"enabled": false` turns the stage off). A view the arm can't reach is
  skipped and listed in the result; it doesn't rule out a grasp. A tilted view can shake loose a weak grip, which is reported as a failure.
- The viewport's red collision badge uses bounding boxes for the 2F-85 linkage and can flag grips that are clear. The simulator's
  check is the accurate one.

Optional planner settings go in `auto_grasp` in `sim_config.json` (fields of `GraspParams`, e.g. `"max_tilt_deg": 45`).

## Taught grasps

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

### Choosing the robot arm

The header has a **Robot: CS66 | EC66** selector (remembered between sessions). The arm does not change the
taught grasp, which is relative to the part. It only decides which arm executes it when you press ▶. The
simulator reports which arm it is building.

| | Elite CS66 | Elite EC66 |
|---|---|---|
| MuJoCo model | `cs66/cs66.xml` | `ec66/ec66.xml`, from Elite's official URDF (`github.com/Elite-Robots/ROS`, `src/elite_description`) |
| Kinematics | | d1 96, a2 418, a3 398, d4 122, d5 98, d6 89 mm (same as RoboDK's `Elite-Robots-EC66.robot`) |
| Joint gains | `kp`/`kv` defaults in `robots.py` | 0.3 × the CS66 gains (the arm is ~3× lighter; the CS66 gains shake the part loose) |

Arms are defined in `robots.py` (model file, flange body and pose, home posture, gains). To add another arm,
add its MJCF there with the first 6 joints being the arm joints. RoboDK `.robot` files are a closed binary
format and can't be loaded; use the manufacturer's URDF/STEP meshes instead.

A saved pose is always approached in a straight line along the TCP axis, with the fingers fully open and closed
to grip. Grasps with a recorded motion, saved or imported from earlier versions, still replay as recorded (`REC` badge),
but new motions can no longer be recorded: use the automatic workflow instead.

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

The MuJoCo window keeps its free, full-workspace view. A live inspection-camera feed appears as a small
inset in the upper-left corner while the simulation runs.

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
the table, several positions) → transfer (part turned back upright in place, then carried in a straight
line; timed so no joint exceeds ~0.5 rad/s) → place in tray → release → retreat (taught path backwards) → home.

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
| `auto_pick_place.py` | Automatic workflow: plan, preview and execute modes (reuses `CadPickPlace` for the scene and stepping) |
| `grasp_planner.py` | Grasp candidates from the part geometry (no robot involved) |
| `gripper_geometry.py` | Where the gripping faces are at each opening, measured from the gripper model |
| `collision_check.py` | Is an arm posture collision-free (part on the table, or carried) |
| `motion_planner.py` | Straight TCP lines and RRT-Connect through free space |
| `test_auto_pick_place.py` | Acceptance test: plans and executes with physics, no window |
| `taught_path.py` | Saved pose or recording → TCP path, finger widths, grip direction (inside/outside) |
| `camera_feed.py` | Inspection camera rendering in a helper process (live `feed.jpg` + stills) |
| `robots.py` | Robot arms (CS66, EC66): model file, flange mount, home posture, joint gains |
| `ec66/` | Elite EC66 MJCF + meshes (from the official URDF, copied as `ec66_description.urdf`) |
| `hande.py` | Robotiq Hand-E MuJoCo model (meshes converted once into `assets/hande/`) |
| `finger_collision.py` | Finger meshes split into convex pieces (CAD scene only), see below |
| `cad_pick_place.py` | The simulator (preview, checks, execution, status reporting) |
| `sim_config.json` | Pick/place positions, lift height, density, grip force, speeds |
| `build_robot.py` | `build(obj='jenga' \| 'cad_part', ...)` – Jenga stays the default |
| `../vite-plugins/mujocoBridge.ts` | Dev-server endpoints `/api/mujoco/simulate`, `/api/mujoco/auto/*` and `/api/mujoco/status` |

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
$PY build_robot.py --object cad_part                  # add --robot ec66 for the EC66

# 3+4. Grasp import + pick & place from an exported JSON (Saved Grasps → Export JSON)
$PY cad_pick_place.py --step ../skf_cam_roller_305703_c-2z_0.stp --grasps ~/Downloads/grasps_skf_cam_roller_305703_c-2z_0_stp.json
#   --grasp-name "Grasp Pose 1"   pick a grasp by name
#   --headless --snapshots out/   no window, PNG per phase
#   --robot ec66                  run it on the Elite EC66 (default: robotId in the request, else cs66)
# A recorded grasp (with "trajectory") in the same JSON is replayed the same way.

# Automatic pick & place, without the web app (each point is x,y,z,nx,ny,nz in the part frame, mm)
$PY auto_pick_place.py --step part.step --point1=0,0,10,0,-1,0 --point2=0,30,10,0,1,0 --mode plan --plan plan.json
$PY auto_pick_place.py --step part.step --plan plan.json --mode preview
$PY auto_pick_place.py --step part.step --plan plan.json --mode execute     # add --headless for no window

# Acceptance test of the automatic workflow (a few minutes, no window)
$PY test_auto_pick_place.py
```
