# Part from the depth camera (instead of a CAD file)

The web app can build the part's model from the RealSense D435i and then let you mark where the fingers grip,
the same way as on a loaded STEP file.

```
RealSense ──> camera_scan_server.py ──> vite-plugins/cameraScanBridge.ts ──> CameraScanPanel (browser)
              (uses live_object_pointcloud.py:        /api/scan/start|stop|proc
               table plane, segmentation,             /api/scan/cam/*  (forwarded)
               height-map model -> STL in mm)
```

## One-time setup

```bash
conda create -n realsense_cam -c conda-forge python=3.11
~/miniforge3/envs/realsense_cam/bin/pip install pyrealsense2 open3d opencv-python numpy
```

The bridge uses `~/miniforge3/envs/realsense_cam/bin/python`. Override it with `CAMERA_PYTHON=/path/to/python`.
The scan server listens on `127.0.0.1:8765` (`SCAN_PORT` to change it). Only the dev server talks to it.

## Use

1. `npm run dev`, open http://localhost:3000, then click **Scan from Camera** in the header.
2. Two feeds appear:
   - **Depth camera** (live): red = near, blue = far, black = no depth, gray = beyond the max depth.
   - **Right view**: **Live object** paints the object's point cloud on the camera image (blue tint = the table,
     yellow box = the object). Once a model is built it switches to **CAD model**, a 3D view of the model
     (drag to rotate, scroll to zoom).
3. Wait until the line under the feeds says `table found` and shows the object size. Use **− / +** to set the max depth
   just behind the part if the room is picked up. Use **Re-fit table** after moving the camera.
4. **Build CAD model**: keep the camera and part still for 30 frames (~1 s). The model is saved as
   `output/scan_<time>.stl` (mm, z = 0 on the table, centered in X/Y) and shown in the right view.
   Use **Rebuild CAD model** if it doesn't look right.
5. **Load model for gripper** puts it in the 3D view as the part. Close the panel (×) to see the whole view, then click
   the model where finger 1 should grip, then where finger 2 should grip (Auto Pick & Place panel).

Closing the panel (×) stops the scan server and releases the camera, so `live_object_pointcloud.py` or
`realsense-viewer` can use it again.

6. **Plan to Points** → **Preview Here** / **Preview Robot** → **Execute Pick & Place** work exactly as for a STEP part
   (see `jenga_mujoco/CAD_SIM.md`). The first plan of a new scan converts its mesh (convex decomposition, ~25 s);
   later runs use the cache.

## How a scanned part is simulated

`jenga_mujoco/step_to_mesh.py` reads `.stl` / `.obj` parts (mm) as well as STEP. The mesh is used as-is for the
visual and the grasp search, CoACD splits it into convex pieces for collisions, and the mass properties come from the
closed mesh. A scan is filled solid and its material is unknown, so mesh parts use `mesh_density_kg_m3` from
`jenga_mujoco/sim_config.json` (2700 = aluminium) instead of the steel `density_kg_m3` used for STEP parts.

## Limits

- The model is a height map: the measured top surface is extruded straight down to the table. Undercuts and the
  hidden back side are filled solid.
- The simulated tray grows to fit the part: it is 130 × 100 mm inside, or a square big enough for the part's footprint
  (plus `tray_margin_m`, 12 mm) when the part is larger.
- The camera is on USB 2 (640×480). A USB 3 port gives 848×480 and less noise.
