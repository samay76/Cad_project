You are a senior frontend engineer. STEP 1 and STEP 2 already work (Three.js + Vite + TypeScript viewer, Robotiq 2F-85 loaded, gripper pose controls, Test Grasp, Save Grasp). Do not rewrite them. Make STEP 3: fix two problems.

## Problem 1: I can only move the gripper, not the CAD model
Add a Part Pose control so I can translate and rotate the CAD model too.
- TransformControls for the CAD model (translate/rotate), with a toggle "Edit: Part | Gripper".
- Numeric panel for the part: X, Y, Z (mm) and Roll, Pitch, Yaw (deg), two-way binding, plus step buttons (+/-X, +/-Y, +/-Z, +/-Rx, +/-Ry, +/-Rz) with adjustable step size, and "Reset Part Pose".
- Add a checkbox "Lock gripper to part" (default ON). When ON, moving or rotating the part moves the gripper with it, so the grasp relative to the part does not change. When OFF, the part moves alone, so I can test how the gripper sits on it.
- IMPORTANT: saved grasps are always stored as T_part_grasp (relative to the part frame). Moving the part in the viewer must never change saved grasps. Keep all pose math in one module and label transforms (T_world_part, T_world_grasp, T_part_grasp).
- Optional button "Set current orientation as part origin": bakes the current part rotation into the model frame. Show a warning that existing saved grasps will be invalid, and ask for confirmation.

## Problem 2 (MAJOR): the gripper goes inside the model instead of holding the outer wall
Real behavior: the fingers close until the finger pads touch the OUTER surface of the part and stop there. The fingers must never penetrate the part. Implement contact-aware gripping.

### 2a. Geometry setup
- Use three-mesh-bvh on the CAD part meshes (computeBoundsTree, accelerated raycast, shapecast / intersectsGeometry) and on the gripper finger meshes. Re-build BVH only when geometry changes.
- Define for the gripper: the closing axis (the line along which the two fingers move, in TCP frame, usually Y), the approach axis (Z), and the finger pad contact surfaces (the inner faces of the two fingertips). Detect these from the finger meshes/joint axes and expose them as configurable values.

### 2b. Close until contact
- Replace the fixed "Close" with: close the fingers in small steps (e.g. 0.5 mm per side) from the current opening, and after each step test whether each finger collides with the part (BVH intersection or shortest distance below a tolerance of 0.1 mm). When a finger touches, that finger stops. Both fingers stopped (or the minimum width reached) = grasp done.
- The final gripper opening is the measured grasp width. Show it in the UI: "Grasp width: xx.x mm", and which fingers touched.
- If the fingers close fully without touching the part, show "Missed: nothing between the fingers". If the part is wider than the maximum opening (85 mm), show "Part too wide for the gripper at this pose".
- Fingers must never visually go through the part. The final state is the fingertips exactly on the surface (gap < 0.1 mm).

### 2c. Initial placement must not start inside the part
- Before closing, the gripper must start OPEN with the fingers outside the part. Add an "Open to clear the part" step: open fully, and if any finger mesh still intersects the part, show a red "Gripper starts inside the part, move the gripper out or change the approach".
- Add a button "Auto-place on outer wall": from the current TCP pose, cast rays along the closing axis in both directions from the TCP to find the two outer surface hits on the part (the near wall on each side). Move the TCP to the midpoint between the two hits (so the part section is centered between the fingers), and report the part thickness at this point. Add the option "Use clicked point as the contact on one finger" which places that finger pad on the clicked surface point instead of centering.
- "Snap to Clicked Point" from Step 2 must now position the gripper so the clicked point is the CONTACT point on the part wall, with the approach pointing toward the part, not put the TCP on the surface. Add a "Standoff from surface" number (default 0 mm) and a "Depth along approach" number (how far the fingertips go down along Z into the grasp region).

### 2d. Updated Test Grasp sequence
Pre-grasp (open, outside the part) -> move straight along approach Z while checking collisions; stop and show red if the open gripper hits the part before reaching the grasp depth -> at grasp depth, close with contact detection (2b) -> lift. Show each phase label and the measured grasp width. If the part is "attached" after closing, move the part together with the gripper during the lift (to visually confirm the grip), then restore.

### 2e. Collision feedback and saving
- Live collision badge: green "No collision" when the gripper bodies (palm, base, finger backs) are not touching the part, and finger pad contact is expected and shown in blue as "Contact". Red "Collision" only for non-pad penetration.
- Highlight penetrating gripper meshes in red.
- Save Grasp now also stores: grasp width (measured), contact points on each finger in the part frame, standoff, depth, approach and closing axes, plus T_part_grasp. Keep Import/Export JSON compatible with old files (default the new fields).

## UI
Keep the light theme, CSS grid and 8px alignment. In the Gripper Pose panel add a section "Contact grip" with: Close (until contact), Open, Auto-place on outer wall, Standoff, Depth, tolerance (mm), grasp width readout, contact status. Add the "Edit: Part | Gripper" toggle and the Part Pose panel in the right sidebar.

## Code structure
src/
  part/PartController.ts       (part transform, lock-to-gripper)
  gripper/ContactGrip.ts       (close-until-contact, auto-place, ray casts)
  gripper/collision.ts         (upgrade to three-mesh-bvh)
  math/poses.ts                (all transforms in one place)
Update the existing GraspController, GripperPanel and Save/Load JSON accordingly.

## Out of scope
Camera, robot communication, belt, inverse kinematics.

## Deliverables
Only new and changed files with full code, new npm packages (three-mesh-bvh), and this acceptance test: load a STEP part, load the gripper, click a wall of the part, press Auto-place on outer wall, press Close (until contact) and see the fingers stop exactly on the outer faces with the grasp width shown, then rotate the part with Lock ON and see the grip stays correct, press Test Grasp and see the part lift with the gripper, then Save Grasp and check the JSON contains the width and contact points.