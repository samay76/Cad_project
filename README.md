# CAD-Based Cobot Pick-Point Tool (Step 2: Gripper & Grasp Pose)

> 📘 **Looking for full architectural details and handoff documentation?**  
> See the complete [STEP2_IMPLEMENTATION_AND_HANDOFF.md](file:///home/human/nferent_cad/STEP2_IMPLEMENTATION_AND_HANDOFF.md) for data flow diagrams, coordinate frames, unit conversion rules, and future developer guidelines.

A high-performance browser-based CAD viewer and robotic pick-point tool with full gripper integration and kinematic grasp pose planning, designed for downstream cobot pick-and-place integration.

![Cobot Pick-Point Gripper Step 2 Screenshot](/home/human/cad-step2-verified.png)

## Tech Stack
- **Vite 6** + **TypeScript** (Vanilla TypeScript, no unnecessary framework overhead)
- **Three.js** (WebGL 3D rendering with OrbitControls and TransformControls)
- **occt-import-js** (OpenCascade compiled to WebAssembly for browser-based STEP / STP / IGES parsing)
- **Three.js STLLoader & OBJLoader** (High-speed parsing of STL and OBJ files)
- **100% Client-Side**: Runs entirely in the browser with zero backend requirements

---

## Features Implemented

### 1. Gripper Asset Serving & Unit Conversion
- **Robotiq 2F-85 Gripper**: Served directly by Vite at `/gripper/` from `/home/human/nferent_cad/robotiq_2f85`.
- **Unit Conversion Centralization**: Robotiq 2F-85 description and STL meshes are defined in **meters**. CAD models operate in **millimeters**. Centralized in `src/config.ts` via `GRIPPER_CONFIG.unitScale = 1000.0`. All meshes and kinematic joint offsets are converted in a single location.
- **"Load Gripper" Button**: In top bar, loads gripper into the scene in slate gray (`#334155`) with metallic guides. Status badge reflects active status.

### 2. Tool Center Point (TCP) Frame
- **Definition**: Origin located at the center between fingertips.
- **Approach Axis**: $+Z$ vector points outward along approach direction (pointing into the grasp target).
- **Visual Indicators**:
  - Small RGB Axes Helper ($30\text{ mm}$ size).
  - Cyan Approach Direction Arrow ($45\text{ mm}$).
  - Tool center sphere.
- **Configurable TCP Offset**: Distance from mounting flange to fingertip center (default $145.0\text{ mm}$), editable dynamically in the UI.

### 3. Gripper Manipulation & Jogging
- **TransformControls Gizmo**: Translate and Rotate modes attached to TCP root.
- **Keyboard Shortcuts**: `T` for Translate mode, `R` for Rotate mode (with input guard).
- **Two-Way Numeric Inputs**:
  - $X, Y, Z$ coordinates in millimeters (part coordinate frame $T_{\text{part\_grasp}}$).
  - $Roll, Pitch, Yaw$ in degrees ($XYZ$ Euler angles).
  - Typing a value moves the gripper; dragging gizmo updates inputs in real-time.
- **Step Jogging**:
  - Jog buttons: $\pm X, \pm Y, \pm Z$ and $\pm Rx, \pm Ry, \pm Rz$.
  - Adjustable step sizes: $1\text{ mm} / 1^\circ$, $5\text{ mm} / 15^\circ$, $10\text{ mm} / 45^\circ$, $25\text{ mm} / 90^\circ$.
- **Surface Snapping & Orientation Controls**:
  - **"Snap to Clicked Point"**: Takes the clicked surface point and face normal $\vec{N}$ on the CAD model, placing the TCP at the surface point with approach vector $+Z_{\text{approach}} = -\vec{N}$.
  - **"Flip Approach (180°)"**: Inverts approach orientation $180^\circ$ around local $X$.
  - **"Spin around Approach"**: $\pm 15^\circ, \pm 90^\circ$ rotations around $+Z_{\text{approach}}$.
  - **"Reset Gripper Pose"**: Restores initial default pose centered above the CAD model.

### 4. Gripper Finger Kinematics (Open / Close)
- Range slider from $0\text{ to }85\text{ mm}$ opening width.
- Quick action buttons: **Close (0mm)**, **Half (42.5mm)**, **Open (85mm)**.
- Real-time numerical readout of current finger opening width.
- Symmetrical 4-bar linkage kinematics on left and right finger groups.

### 5. Interference / Collision Checking
- Hierarchical interference detection between gripper meshes and CAD part meshes.
- Visual feedback: Gripper highlights in warning red (`#ef4444`) on intersection.
- Status badge: `"No collision"` (green) vs `"Collision"` (red).

### 6. Animated "Test Grasp" Motion Sequence
- Pre-grasp $\to$ Approach $\to$ Close $\to$ Lift motion cycle:
  1. **Pre-Grasp**: Retracts along $-Z_{\text{approach}}$ by `preGraspDistance` (default $100\text{ mm}$) and opens fingers to $85\text{ mm}$.
  2. **Approach**: Moves linearly into the target grasp pose along $+Z_{\text{approach}}$.
  3. **Close**: Closes fingers to target opening width onto the part.
  4. **Lift**: Retracts back to pre-grasp standoff position.
  5. **Restore**: Automatically restores the user's editable pose and re-enables gizmos.
- **Phase Labels**: Real-time badge display (`Pre-grasp`, `Approach`, `Close`, `Lift`, `Idle`).
- **Stop Control**: Dedicated Stop button immediately halts animation and restores pose.

### 7. Saved Grasps Management & JSON Export / Import
- **Save Grasp Pose**: Saves grasp name, part-relative `Pose` (position $X, Y, Z$ in mm + quaternion $X, Y, Z, W$), $RPY$ Euler angles, opening width, pre-grasp distance, and TCP offset.
- **Saved Grasps List (Left Sidebar)**:
  - Displays all saved grasps per part.
  - One-click restore to reposition gripper to any saved pose.
  - Inline rename and delete actions.
  - Persistent caching in `localStorage`.
- **JSON Export / Import**:
  - Conforms to cobot `Pose` schema from `src/types.ts`.
  - Downloadable JSON blob for robot controllers.
  - File picker to import existing grasp definitions.

### 8. Viewport Floating Toolbar & Tabbed Sidebar
- Floating toolbar over viewport for fast single-click access:
  `Move (T)`, `Rotate (R)`, `Snap to Point`, `Flip (180°)`, `Test Grasp`, `Stop`.
- Right sidebar tab switcher: `Gripper & Grasp` vs `CAD Properties`.
- Clean 8px spacing, light industrial theme.

---

## Directory Structure
```
nferent_cad/
├── index.html                  # HTML entry point with WASM script tags & font preloads
├── package.json                # Project dependencies & scripts
├── tsconfig.json               # TypeScript configuration
├── vite.config.ts              # Vite configuration (with localFileServer & fs allow)
├── public/
│   ├── occt-import-js.wasm     # OpenCascade WebAssembly binary (7.6MB)
│   ├── occt-import-js.js       # WASM glue script
│   ├── gripper/                # Symlink to robotiq_2f85 assets
│   └── samples/                # Sample CAD test models (.step, .stl, .obj)
├── src/
│   ├── config.ts               # Gripper config & unitScale conversion constant (1000.0)
│   ├── main.ts                 # Application orchestration & event routing
│   ├── types.ts                # TypeScript types (Pose, SavedGrasp, GripperState, etc.)
│   ├── styles.css              # Industrial light-theme styling & responsive grid
│   ├── gripper/
│   │   ├── GripperLoader.ts    # STLLoader mesh assembly with unit scale conversion
│   │   ├── GripperModel.ts     # TCP frame, finger kinematics, materials & helpers
│   │   ├── GraspController.ts  # TransformControls, pose math, snapping & test sequence
│   │   └── collision.ts        # Mesh & Box3 interference detection
│   ├── ui/
│   │   ├── Layout.ts           # Top bar, floating viewport toolbar, tabs & overlays
│   │   ├── GripperPanel.ts     # Right sidebar: pose, jogging, sliders & parameters
│   │   ├── GraspList.ts        # Left sidebar: saved grasps, export & import
│   │   ├── PartsList.ts        # Left sidebar: CAD solid tree & view controls
│   │   └── PropertiesPanel.ts  # Right sidebar: CAD metadata & pick-point readouts
│   └── viewer/
│       ├── Viewer.ts           # Three.js scene, Z-up camera, lighting, helpers & render loop
│       ├── loaders.ts          # STEP (occt-import-js), STL & OBJ loaders
│       └── selection.ts        # Raycasting, pick-point & face normal calculation
└── test/
    └── test-step2-gripper.js   # Automated end-to-end acceptance test suite
```

---

## Setup & Running Locally

1. **Install Dependencies**:
   ```bash
   npm install
   ```

2. **Start Vite Development Server**:
   ```bash
   npm run dev
   ```
   Open `http://localhost:3000` in your browser.

3. **Run Typecheck & Build**:
   ```bash
   npm run build
   ```

4. **Run End-to-End Acceptance Test**:
   ```bash
   node test/test-step2-gripper.js
   ```
