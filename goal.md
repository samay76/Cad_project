You are a senior frontend engineer. Build STEP 1 of a "CAD-based cobot pick-point tool" (similar to Pickit 3D). This step is ONLY a CAD viewer with selection. Do not build anything else.

## Tech stack
- Vite + TypeScript (vanilla, no React unless needed)
- Three.js for 3D rendering
- occt-import-js (OpenCascade WASM) to load .step/.stp/.iges files in the browser
- Three.js STLLoader and OBJLoader for .stl/.obj
- Everything runs locally in the browser; no backend yet

## Features (this step only)
1. **Load CAD button**
   - A "Load CAD File" button that opens the local file picker
   - Accepts .step, .stp, .stl, .obj
   - Also support drag-and-drop onto the viewport
   - Show the loaded file name and a loading indicator; show a clear error message if loading fails

2. **3D viewport**
   - Display the loaded model, auto-centered and scaled to fit the view
   - Mouse controls with OrbitControls: left-drag rotate, right-drag pan, scroll zoom
   - Show a ground grid and an XYZ axes helper (X red, Y green, Z blue)
   - A small orientation gizmo in a corner of the viewport
   - Buttons: "Reset View", "Fit to Model", and a Wireframe on/off toggle
   - Z-up coordinate system (robotics convention)

3. **Selection**
   - Hovering over the model highlights the mesh/part under the cursor
   - Clicking selects a part (STEP files can contain multiple meshes/solids; select by mesh)
   - Selected part gets a distinct highlight color; click empty space to deselect
   - On click, also record the exact clicked surface point and face normal

4. **Properties panel (right side)**
   - File name, number of parts, triangle count
   - Model bounding box size (X, Y, Z) in mm
   - Selected part: name/index
   - Clicked point: X, Y, Z in mm (3 decimals), in the model's coordinate frame
   - Face normal: X, Y, Z (unit vector)
   - A "Copy" button next to the coordinates

## UI / Layout (light theme)
- Clean light theme: white/very light gray background, dark gray text, one accent color (blue)
- Three-block layout using CSS Grid, all blocks aligned with consistent spacing (8px grid) and equal border radius:
  - Top bar: app title on the left, Load CAD button and file name next to it
  - Left sidebar (~260px): Parts list (click an item to select the part) and view controls
  - Center: 3D viewport (takes all remaining space, resizes with window)
  - Right sidebar (~300px): Properties panel
- Subtle borders and soft shadows, system font (Inter if available)
- Light viewport background (e.g. #f5f7fa)
- Everything must be perfectly aligned and responsive down to 1280px width

## Code structure (so future steps can be added)
src/
  main.ts
  viewer/Viewer.ts          (scene, camera, controls, render loop)
  viewer/loaders.ts         (STEP/STL/OBJ loading)
  viewer/selection.ts       (raycasting, hover, click)
  ui/Layout.ts, ui/PropertiesPanel.ts, ui/PartsList.ts
  types.ts                  (define a Pose type: position xyz + quaternion, for later grasp points)
  styles.css

## Out of scope for now
Grasp definition, camera, point clouds, robot communication, backend. Do not add them.

## Deliverables
- Full working code, file by file
- Setup commands (npm create vite, npm install, npm run dev)
- A short README with how to run and how to test
- Acceptance test: load a STEP file, rotate it with the mouse, click a part, and see its X, Y, Z in the panel