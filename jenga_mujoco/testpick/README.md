# testpick — scan-coverage prototype

The EC66 + Robotiq 2F-85 picks the CAD part from the table, holds it in front of a camera on a post, and turns
it view after view. The camera casts a grid of rays. Points on the CAD surface that it sees turn from **red**
to **green**. The loop ends at the target coverage, or when no reachable view adds new points.

## Run

```bash
cd ~/Cad_project
~/miniforge3/envs/mujoco_cad/bin/python jenga_mujoco/testpick/run_testpick.py
```

The first run converts the STEP file (a few seconds) and caches it in `testpick/cache/`.
The window stays open at the end with the final report; close it to exit.

To look through the scan camera, use the viewer's left panel: **Rendering → Camera → scan_cam**.

## Options

| option | what it does |
|---|---|
| `--step "<path>"` | Part to use (STEP/STP, or STL/OBJ in mm). Quote paths with spaces or parentheses. Default: `config.json` → `default_step` (`Cad model/Part Studio 1 - Part 1 (1).step`). |
| `--attach` / `--no-attach` | Default `--attach`: after the fingers close, a weld equality (`testpick_attach`) holds the part to `g_base` at its current pose, so it cannot slip. `--no-attach` uses a friction-only grasp, with `noslip_iterations` as in `auto_pick_place.py`. |
| `--skip-pick` | Skips the pick: the part starts welded in the gripper at the scan point (fast testing). |
| `--up auto\|x\|y\|z` | The part's up axis in its own frame, used to ignore the top and bottom faces. `auto` = world Z of the pose the part settled in on the table. |
| `--include-top-bottom` | Scans the top and bottom faces too, and allows top-down/bottom-up views. |
| `--headless` | No window; runs as fast as possible and prints the same report. |
| `--speed 2` | Simulation speed relative to real time (viewer only). |
| `--seed N` | Seed for the surface sampling and beam selection. |
| `--config file.json` | Another settings file. |

## How it works

1. **Scene** (`scene.py`): calls the project's `build_robot.build()` with the same arguments as
   `cad_pick_place.py`. That gives the same floor, `Table_base` table, tray and light, the EC66 on its mounting
   plate (including `robot_base_offset_mm`), the 2F-85 with prefix `g_` and its coupling plate, the `g_tcp`
   site, and the part on a free joint. The part is placed at `sim_config.json` `pick_xy` exactly as
   `cad_pick_place.py` places it. Through `build()`'s `extra` hook, testpick adds two things:
   - the camera rig `scan_cam_rig` (dark box, lens, post and foot; no collisions) with `<camera name="scan_cam">`
   - an inactive weld `g_base`–`cad_part`
2. **Pick** (`run_testpick.py`): the project's grasp planner (`grasp_planner.generate_candidates`,
   `GripperGeometry`, `SceneCollision`, `MotionPlanner`) picks the first outside grasp that the arm can reach,
   approach and lift without collisions. The arm moves through its position actuators (`d.ctrl[:6]`) with
   cosine-interpolated waypoints.
3. **Coverage** (`coverage.py`):
   - About 1500 points are sampled evenly (`trimesh.sample.sample_surface_even`) on the part's visual mesh, as
     MuJoCo loaded it (`mesh_vert`/`mesh_face`, moved into the body frame with `geom_pos`/`geom_quat`).
   - Points with `|normal · UP| > 0.8` are top/bottom and are ignored; they are drawn as grey dots.
4. **Scan**:
   - `mj_multiRay` casts a 100×75 grid across the camera's field of view, with the camera body excluded.
   - A ray counts only if its first hit is `cad_part_visual`, so the fingers block rays. The part's collision
     hulls and the table's collision boxes have alpha 0, and `mj_ray` never hits those. The geom groups used
     are printed at start.
   - A point turns green when it lies within `HIT_RADIUS` of a hit and faces the camera within 70°.
     `HIT_RADIUS` defaults to 1.2 × the ray spacing at the scan distance.
   - The part pose is re-read before every scan.
5. **Next best view**:
   - Candidates turn the part about its UP axis in 30° steps, combined with tilts of 0°, +25° and −25° towards
     the camera. Views that show the top straight to the camera or the bottom straight up are skipped.
   - For each candidate, the part's centre goes to the scan point. Using the *current* part-to-TCP transform,
     IK gives an arm posture.
   - `mj_multiRay` in a scratch `MjData` predicts how many red points would be visible: the arm at the IK
     solution, the part at the candidate pose, the gripper occluding.
   - The best candidate with a collision-free carry path is executed. A candidate is never repeated.
6. **Report**:
   - coverage % and the number of views
   - how many red points remain
   - how many of those are hidden by the gripper (a short ray from the point along its normal hits a finger)

## Files

| file | contents |
|---|---|
| `run_testpick.py` | entry point; pick, carry and the scan / next-best-view loop |
| `scene.py` | builds the scene (imports the project's builders) + camera rig + attach weld |
| `coverage.py` | surface sampling, ray scan, visibility prediction, report |
| `viz.py` | spheres, beams and view cone in `viewer.user_scn` |
| `config.json` | every setting: scan point, camera pose/fovy, ray grid, radius, angles, target, candidates, speeds |

Settings shared with the main project (`pick_xy`, coupling plate, base offset, grip force, home joints) are read
from `../sim_config.json` and are never written. Nothing outside `testpick/` is modified.

## Placement (config.json)

- Scan point `[0.40, -0.12, 0.30]`: 30 cm above the table top, over the free part of the table.
- Camera: 0.28 m from the scan point, on the +x side (in front of the arm), 45° above it, looking down at it;
  fovy 40°. Its post stands on the table at about x = 0.60 (the table ends at x = 0.625).

Change `scan_point` or `camera.distance_m` / `elevation_deg` / `azimuth_deg` to move them.
