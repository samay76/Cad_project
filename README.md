# Pick cell: teach → simulate → run on the real robot

A web application for one robot cell: an **Elite EC66** arm with a **Robotiq 2F-85** gripper picks CAD parts from a
fixture on the table.

1. **Teach**: load a part's STEP file in the browser. It is shown seated on the pick fixture. Mark the two points the
   fingers should grip; optionally adjust the gripper with Step Jogging or the Pick Offset.
2. **Simulate**: plan the motion and run it in MuJoCo with physics, watched live in the browser.
3. **Run on the real robot**: Real Sim drives the real EC66 and 2F-85 through the same joints, shown live in the
   browser from the real robot's joints.

```
web/        browser app (TypeScript, Three.js, Vite)        -> built to web/dist, served by the server
server/     HTTP API + static web app (FastAPI)              -> python -m server
cell/       simulation, grasp/motion planning, real robot    -> python -m cell.<module>
models/     MuJoCo models of the EC66 and 2F-85, Table_base.step (Git LFS)
cad/        Fixture.step and the part STEP files
config/     cell.json: everything calibrated on the real cell
tests/      pytest suite (planning, sim, real-robot safety, server); web/tests/smoke.mjs browser test
deploy/     install script and systemd service
var/        runtime data, git-ignored: converted-mesh cache and one folder per run (var/runs/<id>/)
```

## Install (cell PC, Ubuntu)

Needs miniforge (conda), Node 20 (`web/.nvmrc`), git-lfs, and an X desktop session for the MuJoCo windows.

```bash
git lfs install && git lfs pull           # models/Table_base.step (39 MB)
deploy/install.sh                         # Python env 'pick-cell', web build, .env, config check, systemd service
```

Then open <http://127.0.0.1:8000>. Manage the service with `systemctl --user status|restart|stop pick-cell`; its log:
`journalctl --user -u pick-cell -f`. Each run also has `var/runs/<id>/log.txt` and `status.json`.

The 2F-85 needs access to the USB adapter: add the user to the `dialout` group (or `sudo chmod a+rw /dev/ttyUSB0`).

Machine settings are in `.env` (see `.env.example`): host/port, Python, data folder, how many runs to keep, DISPLAY.
`CELL_HOST=0.0.0.0` makes the app reachable from other PCs on the cell network. There is no login: only do that on
a closed cell network.

## Develop

```bash
conda activate pick-cell
python -m server                          # API on :8000 (serves web/dist if built)
cd web && npm run dev                     # http://localhost:3000, /api forwarded to :8000

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest      # ~3 min, no hardware, no windows
cd web && npm run build && npm run smoke  # browser: load part -> fixture -> mark points -> plan (SMOKE_SIM=1: + Sim)
python -m cell.config_check               # validate config/cell.json
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` keeps ROS's pytest plugins (from a sourced ROS `PYTHONPATH`) out of the tests.
The server and its jobs ignore that `PYTHONPATH` already.

## Using it

In the **Auto Pick & Place** panel (left):

1. **Load CAD** (`.step`/`.stp`, or `.stl`/`.obj` in mm). The part appears on the pick fixture as in the simulator.
2. Click where one finger should grip, then where the other should. The surfaces decide the grip:
   - facing away from each other (two outer sides): **outside grip**, the fingers close on the points
   - facing each other (two sides of a bore or slot): **inside grip**, the fingers open against the points
   - same direction, or more than 53° from opposite: refused, the fingers would slide off

   The gripper goes exactly where you marked or nowhere: if the points can't be reached, the app says why.
3. **Plan to Points**: up to three solutions, best first. Click one to show it.
   **Step Jogging** (Gripper tab) or **Pick Offset** after planning, then Plan again: the arm plans to exactly that
   gripper pose, or the app says why it can't.
4. **Preview Here** / **Preview Robot** (no physics), then **Sim** (full physics). Sim reports how close each
   finger came to its point. Running Sim to the end enables **Real Sim** for that plan.
5. **Go Home (real robot)**, then **Real Sim**. **Stop** sends `stop` to the robot. Keep a hand on the E-stop.

### Live view

Preview Robot, Sim, Real Sim and Go Home are shown **live in the 3D view**: the same table, fixture, tray, EC66,
2F-85 and part that MuJoCo simulates, moving as the job runs (Real Sim: as the real robot reports its joints). The
teaching part, fixture and gripper are hidden meanwhile; the box at the bottom left shows the live step and the
result, and **Back to teaching** returns to marking points (Plan to Points or Clear Points also do).

How it works: the job writes the scene once (`var/runs/<id>/scene.json`, meshes in `var/cache/webview/`, which the
browser caches) and appends the moving bodies' poses ~30 times a second (`frames.bin`); the page polls them and plays
them ~0.15 s behind, interpolated (`cell/web_view.py`, `web/src/autopick/LiveView.ts`).
`web_view` in `config/cell.json`: `mujoco_window: true` also opens the MuJoCo window on the cell PC.

### Grasp test or full pick & place

`config/cell.json` → `grasp_test.enabled`:

- `true`: go to the pick point, wait `wait_before_close_s` (5 s), close on the part in the fixture, hold
  `hold_closed_s` (10 s), open, back out along the approach line, go home. The part stays in the fixture.
  This is for measuring the sim-to-real gap.
- `false`: pick, lift `lift_height_m`, carry to the tray at `place_xy`, release and go home.

### Parts and the fixture

`pick_station` in `config/cell.json`: `Fixture.step` is bolted over the corner hole near `near_xy`.
With `seat_all_parts: true`, every part is placed on it by its CAD coordinates, so **each part's STEP must be
exported in the fixture's coordinate frame** (e.g. from the assembly with the fixture). A part that would float above
the fixture or sit below the table is refused with a message saying so. The part's turn about the vertical axis
from the Part Pose tab is kept; tilting it is refused. The fixture holds the part until the gripper has closed.

### Real robot

- EC66 controller at `192.168.1.200` (JSON-RPC port 8055), pendant in **REMOTE** mode, robot stopped (no alarm).
  IP `fake` in the panel runs everything against a simulated robot.
- Speed 1–10 % (hard cap 10 %). Before moving, the whole path is checked: joint limits, no joint turning more
  than 120° in one move, wrist and TCP clear of the table and the pendant holder. Joints a whole turn away from
  the plan (e.g. J5 271° vs −89°) are kept on the robot's current turn.
- Every waypoint must be reached within 1°, or the run stops. Every stop is logged (`real_stops` in the run's
  `status.json`): target vs reached joints, error and time. The real gripper's closed width (and whether it felt the
  part) is also logged, for comparing sim and real.
- **Real gripper (2F-85 on USB)** box: `real_gripper` in the config (port, speed 64, force 50). Without it the gripper
  only moves in the sim.
- `debug_pause_at_pick_s` (config): full pick & place only, a pause at the pick point before closing (0 = off).

## Calibration (`config/cell.json`)

| key | what | how |
|---|---|---|
| `pick_motion.home_joints_deg.ec66` | home pose (pendant degrees) | read from the robot at the wanted pose |
| `robot_base_offset_mm.ec66` | real robot base vs the table CAD, mm | `python -m cell.calibrate_robot` (below) |
| `gripper_coupling_mm.ec66.2f85` | coupling plate flange → gripper (22 mm) | measured |
| `pick_station.near_xy`, `yaw_deg` | where the fixture is bolted | table hole grid |
| `place_xy` | tray centre | measured |

Base offset: `python -m cell.calibrate_robot --close-gripper`. Then jog the closed fingertips onto the centre of big
table hole (15, 0) with the pendant, and run `python -m cell.calibrate_robot` (reads the joints, prints the error).
`--write` saves the correction. Check at a second hole (`--hole 2 12`): both should be within 1–2 mm. Plan again
afterwards. `python -m cell.show_locations` prints where the pick and drop are on the table.

The server checks `config/cell.json` at start-up (`cell/config_check.py`) and does not start on a bad value.

## How it works

- `cell/cad_pick_place.py`: the scene (EC66 + 2F-85 + table + tray + fixture + part), stepping, viewer, ghost
  gripper. `step_to_mesh.py` converts STEP files to meshes and convex collision pieces (CoACD), cached in
  `var/cache/` (first run of a part: ~30 s).
- `cell/auto_pick_place.py`: the job entry point (`--mode plan|preview|execute|real|home`), grasp from the two points
  (`grasp_planner.py`, `gripper_geometry.py`), motion (`ik.py`, `motion_planner.py`: straight TCP lines and
  RRT-Connect), collision checks against the physics shapes (`collision_check.py`).
- `cell/web_view.py`: the live view stream (scene + body poses) for the web app.
- `cell/real_robot.py`, `cell/real_gripper.py`: the EC66 client, waypoint reduction, safety checks, the 2F-85 on
  Modbus RTU.
- Frames (`cell/frames.py`): the web tool and MuJoCo use the same part frame (STEP native, mm in the web, m in MuJoCo)
  and the same TCP (+Z approach, X closing). `T_world_tcp = T_world_part · T_part_tcp`.
- `server/`: one job at a time as a child process (`python -m cell.auto_pick_place ...`). Stop = SIGTERM; in real
  mode the job then sends `stop` to the robot. Run folders beyond `CELL_KEEP_RUNS` are deleted.

## Troubleshooting

| message | meaning |
|---|---|
| `The robot is in TEACH mode` | switch the pendant to REMOTE |
| `Real move refused ... a joint would turn N deg` | the robot is far from home: press Go Home first |
| `The robot stopped N deg away from ...` | the robot did not reach a waypoint (blocked, or stopped from the pendant) |
| `The gripper did not finish ...` | 2F-85 not responding: check USB, power, port permission |
| `... not drawn in the fixture's coordinate frame` | export the part's STEP in the fixture's frame |
| `The two surfaces face the same way` | mark points on opposite faces (two sides of a wall or bore) |
| `config/cell.json has problems` | the server lists the bad keys; fix them and restart |
