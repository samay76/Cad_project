# CAD-Based Cobot Pick-Point Tool (Step 1: CAD Viewer with Selection)

A high-performance browser-based CAD viewer with part hierarchy selection and 3D pick-point surface recording, designed as Step 1 of a robotics cobot pick-and-place workflow (similar to Pickit 3D).

![Cobot Pick-Point CAD Viewer Screenshot](/home/human/.gemini/antigravity-cli/brain/5adce33d-880f-4878-a233-329288455ba8/cad-browser-verified.png)

## Tech Stack
- **Vite 6** + **TypeScript** (Vanilla TypeScript, no unnecessary framework overhead)
- **Three.js** (WebGL 3D rendering with OrbitControls and feature edge extraction)
- **occt-import-js** (OpenCascade compiled to WebAssembly for browser-based STEP / STP / IGES parsing)
- **Three.js STLLoader & OBJLoader** (High-speed parsing of STL and OBJ files)
- **100% Client-Side**: Runs entirely in the browser with zero backend requirements

---

## Features Implemented

### 1. Load CAD & Drag-and-Drop
- **"Load CAD File" Button**: Opens the native file picker accepting `.step`, `.stp`, `.stl`, `.obj`, `.iges`, `.igs`.
- **Drag-and-Drop**: Drop files anywhere directly onto the 3D viewport with interactive dashed drop zone overlay.
- **Sample CAD Models**: Quick-access dropdown menu to load built-in sample models (`as1_pe_203.stp` assembly, `cube.stp`, `cube.stl`, `cube.obj`).
- **Loading & Error Feedback**: Animated progress spinner during WASM geometry triangulation; friendly toast notifications with error diagnostics if parsing fails.

### 2. 3D Viewport & Robotics Coordinate Frame
- **Z-Up Convention**: Aligned with robotics industry standards (`camera.up = (0, 0, 1)` and ground grid in the XY plane).
- **Auto-Centering & Fit**: Loaded CAD models are automatically framed and centered without mutating native millimeter coordinates.
- **Controls**:
  - `Left-Click Drag`: Orbit / Rotate
  - `Right-Click Drag`: Pan
  - `Scroll Wheel`: Zoom
- **Grid & XYZ Axes Helper**: Ground grid with subtle millimeter divisions and standard colored axes helper (**X** Red, **Y** Green, **Z** Blue).
- **Orientation Gizmo**: Synchronized 3D axis gizmo in the top-right corner.
- **Camera Presets & View Controls**:
  - **Fit Model**: Smoothly reframes the camera to the full model bounding box.
  - **Reset View**: Resets camera to default isometric view angle.
  - **Camera Presets**: Fast 1-click snapping to `ISO`, `TOP`, `FRT` (Front), `RGT` (Right).
  - **Wireframe Toggle**: Toggles full triangle tessellation on/off.

### 3. Selection & Surface Pick-Point Recording
- **Hover Highlighting**: Dynamically illuminates meshes under cursor in sky blue glow.
- **Part Selection**:
  - Click any part/solid mesh in the 3D viewport or click any item in the left sidebar's **Parts List**.
  - Selected part receives distinctive bright blue highlight with enhanced edge outlines.
  - Clicking empty background deselects the part.
- **Surface Pick-Point & Normal Calculation**:
  - Computes the exact clicked 3D surface point `(X, Y, Z)` in the **model's coordinate frame** in millimeters to 3 decimal places (`0.001 mm`).
  - Computes the exact surface face normal unit vector `[Nx, Ny, Nz]`.
  - Places a 3D pick-point indicator (sphere + normal vector arrow) directly on the surface in the scene.

### 4. Properties Panel (Right Sidebar)
- **Model Overview**: File name, format badge, total parts count, total triangle count.
- **Bounding Box Dimensions**: Model size `X`, `Y`, `Z` in millimeters.
- **Selected Part Info**: Part name, solid index `#`.
- **Surface Pick Point Card**:
  - Formatted coordinates with axis tags: `X: ... mm`, `Y: ... mm`, `Z: ... mm`.
  - Face normal unit vector: `Nx`, `Ny`, `Nz`.
  - **One-Click "Copy" Buttons**: Copies coordinates directly to clipboard with visual `"Copied!"` confirmation.

### 5. Clean Light Theme & 8px Grid Layout
- Clean industrial design system with white cards, subtle borders, soft shadows, and accent blue (`#2563eb`).
- Responsive 3-block grid layout: Left Sidebar (`260px`), Center Viewport (flexible `1fr`), Right Sidebar (`300px`).
- System typography (`Inter` / `JetBrains Mono` for coordinates).

---

## Directory Structure
```
nferent_cad/
├── index.html                  # HTML entry point with WASM script tags & font preloads
├── package.json                # Project dependencies & scripts
├── tsconfig.json               # TypeScript configuration
├── vite.config.ts              # Vite configuration
├── public/
│   ├── occt-import-js.wasm     # OpenCascade WebAssembly binary (7.6MB)
│   ├── occt-import-js.js       # WASM glue script
│   └── samples/                # Sample CAD test models (.step, .stl, .obj)
│       ├── as1_pe_203.stp      # Multi-part STEP assembly (18 solids)
│       ├── cube.stp            # Simple STEP cube
│       ├── cube.stl            # STL mesh
│       └── cube.obj            # OBJ model
├── src/
│   ├── main.ts                 # Application orchestration & event routing
│   ├── types.ts                # TypeScript types (including cobot Pose definition)
│   ├── styles.css              # Industrial light-theme styling & responsive grid
│   ├── ui/
│   │   ├── Layout.ts           # App layout, top bar, drag-and-drop & loading overlays
│   │   ├── PartsList.ts        # Left sidebar: view controls & interactive parts list
│   │   └── PropertiesPanel.ts  # Right sidebar: metadata, bounding box & pick-point readouts
│   └── viewer/
│       ├── Viewer.ts           # Three.js scene, Z-up camera, lighting, helpers & render loop
│       ├── loaders.ts          # STEP (occt-import-js), STL & OBJ loaders
│       └── selection.ts        # Raycasting, hover/select state, pick point & normal calculation
└── test/
    ├── verify.js               # Node.js test suite for loaders, bounding box, raycasting & Pose
    └── e2e-browser-test.js     # Live headless browser end-to-end automation test
```

---

## Setup & Running Locally

### 1. Installation
```bash
# Clone or navigate to the repository
cd /home/human/nferent_cad

# Install dependencies
npm install
```

### 2. Development Server
```bash
npm run dev
```
Open [http://localhost:5173](http://localhost:5173) in your browser.

### 3. Production Build & Preview
```bash
npm run build
npm run preview
```

---

## Automated Verification Tests

### Unit & Math Verification
Runs tests validating STEP parsing, assembly part hierarchy, bounding box mm calculation, STL/OBJ loading, raycasting, normal vector normalization, and Pose data structure:
```bash
node test/verify.js
```
Expected output:
```
Test Results: 14 Passed, 0 Failed
```

### Live Browser End-to-End Verification
Spins up headless browser automation via WebDriver to verify model rendering, parts list population (18 parts), part selection, pick-point readout, and screenshot capture:
```bash
node test/e2e-browser-test.js
```

---

## Acceptance Test Checklist
1. **Load a STEP File**:
   - On load, `as1_pe_203.stp` loads automatically, showing 18 parts in the left sidebar and bounding box `5080.000 × 2209.800 × 3810.000 mm` in the right panel.
   - Alternatively, click **"Load CAD File"** or select **"Samples -> Simple Cube (STEP)"**.
2. **Rotate Viewport**:
   - Left-drag rotates the model around the target.
   - Right-drag pans the camera.
   - Scroll wheel zooms in and out.
3. **Click a Part**:
   - Hover over any part: see it highlight in blue glow.
   - Click a part (or click an item in the Parts List): the part highlights distinctly.
   - Look at the **Surface Pick Point** section in the right sidebar:
     - Exact `X, Y, Z` in mm (3 decimal places) in the model's coordinate frame is displayed.
     - Face normal `[Nx, Ny, Nz]` unit vector is displayed.
     - Click **"Copy"** next to coordinates: copies to clipboard with green checkmark confirmation.
