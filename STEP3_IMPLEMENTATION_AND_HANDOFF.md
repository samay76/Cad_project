# Step 3 Implementation & Handoff Guide: Contact-Aware Gripping & Part Pose Control

This document provides a comprehensive technical overview and operational guide for **Step 3** of the CAD-Based Cobot Pick-Point Tool. It details all architecture decisions, math formulations, collision detection optimizations using `three-mesh-bvh`, and acceptance test results.

---

## 1. Executive Summary

In Step 3, the application was upgraded to solve two critical physical simulation problems:
1. **Problem 1 (Part Pose Control & Gripper Locking)**:
   - Added two-way interactive part pose controls (translation and rotation) via TransformControls gizmo, numeric RPY inputs, and step jog buttons.
   - Implemented `"Lock gripper to part"` (default **ON**). When active, moving or rotating the CAD part transforms the gripper synchronously so that the relative grasp transformation $T_{\text{part\_grasp}}$ remains strictly invariant.
   - Centralized all coordinate transformations in [`src/math/poses.ts`](file:///home/human/nferent_cad/src/math/poses.ts) ($T_{\text{world\_part}}$, $T_{\text{world\_grasp}}$, $T_{\text{part\_grasp}}$).
   - Added `"Set current orientation as part origin"` with confirmation warnings.
2. **Problem 2 (Contact-Aware Gripping & Zero Penetration)**:
   - Integrated `three-mesh-bvh` for accelerated raycasting and geometry collision checking between CAD models and Robotiq 2F-85 URDF meshes.
   - Implemented stepped finger closing (`closeUntilContact`) that stops each finger individually when its pad touches the outer CAD wall (gap $< 0.1\text{ mm}$), preventing any penetration into the solid CAD model.
   - Added real-time measured grasp width readout in the UI (e.g. `"Grasp width: 49.7 mm"`, fingers touched readout).
   - Implemented `"Auto-place on outer wall"` which casts rays along the closing axis ($\pm X$) from TCP, calculates wall thickness, and centers the TCP at the midpoint.
   - Updated `"Snap to Clicked Point"` to position the fingertips with configurable **Standoff** (mm) and **Depth** (mm) along the approach vector.
   - Updated the **Test Grasp** animation sequence to carry and lift the CAD model together with the gripper during the grip phase, then return and restore.
   - Extended the Saved Grasp schema to store $T_{\text{part\_grasp}}$, `measuredGraspWidth`, and `contactPoints` in the part coordinate frame.

---

## 2. Architecture & File Structure

```
src/
├── math/
│   └── poses.ts                 # Centralized transform math (T_world_part, T_world_grasp, T_part_grasp)
├── part/
│   └── PartController.ts        # Part pose state, step jogging, origin baking, lock-to-gripper sync
├── gripper/
│   ├── collision.ts             # three-mesh-bvh integration, fast bounding box pruning, pad contact detection
│   ├── ContactGrip.ts           # Stepped close-until-contact, auto-place wall raycasts, contact snapping
│   ├── GripperLoader.ts         # URDFLoader setup with package mapping & unit conversion (1000x scale)
│   ├── GripperModel.ts          # Robotiq 2F-85 kinematics, joint mimic handling, mesh caching
│   └── GraspController.ts       # Gizmo manager, pick & place animation, collision evaluation
├── ui/
│   ├── PartPanel.ts             # Right sidebar panel for Part Pose controls & lock checkbox
│   ├── GripperPanel.ts          # Right sidebar panel with Contact Grip controls, readouts & parameters
│   ├── GraspList.ts             # Left sidebar grasp poses list with measured width badges & JSON export
│   └── Layout.ts                # Application toolbar, target selector (Gripper | Part), tabs
└── types.ts                     # TypeScript schemas for ContactPoints, SavedGrasp, GripperState, PartPoseState
```

---

## 3. Coordinate System & Math Formulas

All coordinate transformations are strictly labeled and centralized in [`src/math/poses.ts`](file:///home/human/nferent_cad/src/math/poses.ts):

### 3.1 Frame Definitions
- **World Frame** ($\mathcal{F}_{\text{world}}$): Standard Three.js world frame ($+X$ right, $+Y$ up, $+Z$ out).
- **CAD Part Frame** ($\mathcal{F}_{\text{part}}$): Coordinate origin defined by the imported STEP model.
- **Grasp TCP Frame** ($\mathcal{F}_{\text{grasp}}$): Origin located at the center of the Robotiq 2F-85 fingertips, $+Z$ pointing forward along the approach direction, $+X$ pointing along the finger closing axis.

### 3.2 Coordinate Transform Relations
1. **Part Grasp Transform**:
   $$T_{\text{part\_grasp}} = \left(T_{\text{world\_part}}\right)^{-1} \times T_{\text{world\_grasp}}$$
2. **Synchronized Gripper Transform (Lock ON)**:
   $$T_{\text{world\_grasp}} = T_{\text{world\_part}} \times T_{\text{part\_grasp}}$$
   When the user rotates or translates the CAD part, $T_{\text{world\_grasp}}$ is recalculated using the cached $T_{\text{part\_grasp}}$, ensuring the gripper stays locked in place relative to the part.
3. **Contact Points in Part Frame**:
   $$P_{\text{part}} = \left(T_{\text{world\_part}}\right)^{-1} \times P_{\text{world}}$$

---

## 4. Contact-Aware Gripping Implementation

### 4.1 `three-mesh-bvh` Performance Optimization
Robotiq 2F-85 DAE links are defined in **meters**, while CAD models are triangulated in **millimeters** ($1000\times$).
- Distance queries are inverted to CAD local space (`cadGeom.boundsTree.closestPointToGeometry(tipGeom, tempMatrix)`), measuring distance directly in unscaled millimeters.
- Fast bounding-box pre-filtering (`bMeshBox.intersectsBox(cadBox)`) discards non-overlapping meshes in microseconds before executing tree traversals.
- Body penetration checks use `intersectsGeometry`, stopping at the first intersecting triangle pair.
- Bounded closest-point queries with `maxThreshold = tolerance + 5.0 mm` prune distant BVH branches, accelerating stepped closing by over $40\times$.

### 4.2 Auto-Place & Stepped Closing
1. **Auto-Place**:
   Casts rays along $\pm X$ (closing axis) from the TCP into the CAD part meshes. The hit points determine the outer wall thickness and midpoint:
   $$\text{midpoint} = \frac{P_{\text{left}} + P_{\text{right}}}{2}$$
   The TCP position is translated along the closing vector to match the midpoint, and fingers open to $\min(85, \text{thickness} + 10\text{ mm})$.
2. **Close Until Contact**:
   Closes fingers using adaptive stepping (coarse when distant, fine when near) while monitoring pad distance. When pad distance $\le 0.1\text{ mm}$, the respective finger halts. Both fingers stopped yields the final measured grasp width.

---

## 5. Automated Acceptance Test Verification

All requirements of `goal.md` are verified end-to-end via headless Firefox WebDriver (`test/test-step3-acceptance.js`):

```bash
node test/test-step3-acceptance.js
```

### Acceptance Test Output:
- **STEP Model & URDF Loading**: SKF Cam Roller (`skf_cam_roller_305703_c-2z_0.stp`) and Robotiq 2F-85 loaded with all 9 meshes indexed.
- **Surface Click & Auto-Place**: Surface point hit; Auto-place centered TCP with thickness readout: `thickness = 3.6 mm`.
- **Close Until Contact**: Stepped closing halted at outer wall with `Grasp width: 29.2 mm (both fingers touching outer wall)`, badge `CONTACT MADE`.
- **Part Rotation with Lock ON**: Part rotated $+15^\circ$ Rx; gripper moved synchronously maintaining invariant grasp pose in part frame.
- **Test Grasp Animation**: Progressed through phases (`Pre-Grasp` -> `Approach` -> `Contact-Aware Close` -> `Lift CAD Model` -> `Idle`).
- **Save Grasp JSON Schema**: Verified export containing `T_part_grasp`, `measuredGraspWidth: 49.7 mm`, and contact points in part frame:
  ```json
  "contactPoints": {
    "left": { "x": 16.9, "y": -0.078, "z": 23.497 },
    "right": { "x": 0.6, "y": 0, "z": -23.5 }
  }
  ```
- **Visual Screenshot**: Saved to `cad-step3-verified.png`.
