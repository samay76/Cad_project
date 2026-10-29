import * as THREE from 'three';
import type { GripperModel } from './GripperModel';
import { CollisionChecker } from './collision';
import { PoseMath } from '../math/poses';
import type { ContactPoints, Vector3D } from '../types';
import { GRIPPER_CONFIG } from '../config';

export interface CloseUntilContactResult {
  status: 'contact' | 'missed' | 'too-wide' | 'collision';
  message: string;
  measuredGraspWidth: number;
  leftContact: boolean;
  rightContact: boolean;
  contactPointsWorld: {
    left?: THREE.Vector3;
    right?: THREE.Vector3;
  };
  contactPointsPart: ContactPoints;
}

export interface AutoPlaceResult {
  success: boolean;
  thickness: number;
  message: string;
  midpointWorld?: THREE.Vector3;
}

export class ContactGrip {
  private static raycaster = new THREE.Raycaster();

  /**
   * 2b. Close Until Contact:
   * Stepped finger closing (0.5 mm steps down to contact, gap < 0.1 mm tolerance).
   * Stops when finger pad touches the outer surface of the part.
   * Prevents any penetration into the CAD solid.
   */
  public static closeUntilContact(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null,
    tolerance = 0.1 // mm
  ): CloseUntilContactResult {
    if (!cadPartsRoot) {
      return {
        status: 'missed',
        message: 'No CAD model loaded to grasp',
        measuredGraspWidth: gripperModel.getOpeningWidth(),
        leftContact: false,
        rightContact: false,
        contactPointsWorld: {},
        contactPointsPart: {}
      };
    }

    CollisionChecker.ensureBVH(cadPartsRoot);

    // Initial check: if already at max opening (85mm) and intersecting CAD part body
    const initialState = CollisionChecker.evaluateState(gripperModel, cadPartsRoot, tolerance);
    if (initialState.hasCollision) {
      return {
        status: 'too-wide',
        message: 'Part too wide for the gripper at this pose (body collision)',
        measuredGraspWidth: gripperModel.getOpeningWidth(),
        leftContact: initialState.leftPadContact,
        rightContact: initialState.rightPadContact,
        contactPointsWorld: {
          left: initialState.leftContactWorld,
          right: initialState.rightContactWorld
        },
        contactPointsPart: this.extractPartFrameContacts(initialState, cadPartsRoot)
      };
    }

    // Step closing down from current opening in 0.5 mm steps
    let currentWidth = gripperModel.getOpeningWidth();
    const stepSize = 0.5; // mm
    const minWidth = GRIPPER_CONFIG.minOpeningWidth;

    let leftStopped = false;
    let rightStopped = false;
    let leftStopAngle = 0;
    let rightStopAngle = 0;

    let lastSafeWidth = currentWidth;
    let finalState = initialState;

    // Evaluate initial state at current opening
    if (initialState.leftPadContact) {
      leftStopped = true;
      leftStopAngle = (1.0 - currentWidth / GRIPPER_CONFIG.maxOpeningWidth) * 0.8;
    }
    if (initialState.rightPadContact) {
      rightStopped = true;
      rightStopAngle = (1.0 - currentWidth / GRIPPER_CONFIG.maxOpeningWidth) * 0.8;
    }

    while (currentWidth > minWidth && (!leftStopped || !rightStopped)) {
      // Adaptive step size: coarse (1.5mm) when distant, fine (0.2mm) when close
      const isClose = (finalState.leftPadDist <= 2.5 || finalState.rightPadDist <= 2.5);
      const stepSize = isClose ? 0.2 : 1.5;

      const nextWidth = Math.max(minWidth, currentWidth - stepSize);
      const targetAngle = (1.0 - nextWidth / GRIPPER_CONFIG.maxOpeningWidth) * 0.8;

      const angleL = leftStopped ? leftStopAngle : targetAngle;
      const angleR = rightStopped ? rightStopAngle : targetAngle;

      gripperModel.setFingerAngles(angleL, angleR);
      gripperModel.root.updateMatrixWorld(true);

      const state = CollisionChecker.evaluateState(gripperModel, cadPartsRoot, tolerance);

      // If gripper body penetrates, we cannot close further
      if (state.hasCollision) {
        break;
      }

      if (!leftStopped && (state.leftPadContact || state.leftPadDist <= tolerance)) {
        leftStopped = true;
        leftStopAngle = targetAngle;
      }
      if (!rightStopped && (state.rightPadContact || state.rightPadDist <= tolerance)) {
        rightStopped = true;
        rightStopAngle = targetAngle;
      }

      finalState = state;
      lastSafeWidth = nextWidth;
      currentWidth = nextWidth;

      if (leftStopped && rightStopped) {
        break;
      }
    }

    // Measure final distance between left and right fingertips or use opening width
    let measuredWidth = Number(lastSafeWidth.toFixed(1));
    if (finalState.leftContactWorld && finalState.rightContactWorld) {
      measuredWidth = Number(
        finalState.leftContactWorld.distanceTo(finalState.rightContactWorld).toFixed(1)
      );
    }

    gripperModel.setOpeningWidth(lastSafeWidth);
    gripperModel.root.updateMatrixWorld(true);

    const contactPointsPart = this.extractPartFrameContacts(finalState, cadPartsRoot);

    if (leftStopped && rightStopped) {
      return {
        status: 'contact',
        message: `Grasp width: ${measuredWidth} mm (both fingers touching outer wall)`,
        measuredGraspWidth: measuredWidth,
        leftContact: true,
        rightContact: true,
        contactPointsWorld: {
          left: finalState.leftContactWorld,
          right: finalState.rightContactWorld
        },
        contactPointsPart
      };
    } else if (leftStopped || rightStopped) {
      const fingerName = leftStopped ? 'left' : 'right';
      return {
        status: 'contact',
        message: `Grasp width: ${measuredWidth} mm (${fingerName} finger touching wall)`,
        measuredGraspWidth: measuredWidth,
        leftContact: leftStopped,
        rightContact: rightStopped,
        contactPointsWorld: {
          left: finalState.leftContactWorld,
          right: finalState.rightContactWorld
        },
        contactPointsPart
      };
    } else {
      return {
        status: 'missed',
        message: 'Missed: nothing between the fingers',
        measuredGraspWidth: measuredWidth,
        leftContact: false,
        rightContact: false,
        contactPointsWorld: {},
        contactPointsPart: {}
      };
    }
  }

  /**
   * 2c. Open to Clear:
   * Opens the gripper fully (85 mm).
   * If any finger mesh still intersects the part, warns that gripper starts inside the part.
   */
  public static openToClear(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null
  ): { cleared: boolean; message: string } {
    gripperModel.setOpeningWidth(GRIPPER_CONFIG.maxOpeningWidth);
    gripperModel.root.updateMatrixWorld(true);

    if (!cadPartsRoot) {
      return { cleared: true, message: 'Gripper fully open (85 mm)' };
    }

    const state = CollisionChecker.evaluateState(gripperModel, cadPartsRoot, 0.05);
    if (state.hasCollision) {
      return {
        cleared: false,
        message: 'Gripper starts inside the part, move the gripper out or change the approach'
      };
    }

    return {
      cleared: true,
      message: 'Gripper fully open and clear of CAD part'
    };
  }

  /**
   * 2c. Auto-place on outer wall:
   * From the current TCP pose, cast rays along the closing axis (+/- X in TCP frame)
   * to find the outer surface hits on the part. Centers TCP at midpoint between walls.
   */
  public static autoPlaceOnOuterWall(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null,
    clickedPoint?: THREE.Vector3,
    useClickedPointOnOneFinger = false
  ): AutoPlaceResult {
    if (!cadPartsRoot) {
      return { success: false, thickness: 0, message: 'No CAD model loaded' };
    }

    CollisionChecker.ensureBVH(cadPartsRoot);
    const cadMeshes = CollisionChecker.getCadMeshes(cadPartsRoot);
    if (cadMeshes.length === 0) {
      return { success: false, thickness: 0, message: 'No CAD meshes found' };
    }

    // Local closing axis in world coordinates (X axis of TCP)
    const tcpPos = gripperModel.root.position.clone();
    const tcpRot = gripperModel.root.quaternion.clone();
    const closingDirWorld = new THREE.Vector3(1, 0, 0).applyQuaternion(tcpRot).normalize();

    // 1. Raycast along closing axis in +X and -X directions
    const rayPositive = new THREE.Raycaster(tcpPos, closingDirWorld, 0.1, 200);
    const rayNegative = new THREE.Raycaster(tcpPos, closingDirWorld.clone().negate(), 0.1, 200);

    const hitsPos = rayPositive.intersectObjects(cadMeshes, false);
    const hitsNeg = rayNegative.intersectObjects(cadMeshes, false);

    // If TCP is inside or outside, also test a line segment spanning across the gripper opening
    let hitLeft: THREE.Vector3 | null = null;
    let hitRight: THREE.Vector3 | null = null;

    if (hitsPos.length > 0 && hitsNeg.length > 0) {
      // TCP is between the two walls
      hitRight = hitsPos[0].point;
      hitLeft = hitsNeg[0].point;
    } else {
      // Cast across the full opening from outside (+X to -X and -X to +X)
      const spanOriginPos = tcpPos.clone().addScaledVector(closingDirWorld, 60);
      const spanRayInwards = new THREE.Raycaster(
        spanOriginPos,
        closingDirWorld.clone().negate(),
        0.1,
        120
      );
      const spanHits = spanRayInwards.intersectObjects(cadMeshes, false);

      if (spanHits.length >= 2) {
        hitRight = spanHits[0].point;
        hitLeft = spanHits[spanHits.length - 1].point;
      } else if (spanHits.length === 1 && clickedPoint) {
        hitRight = spanHits[0].point;
        hitLeft = clickedPoint.clone();
      }
    }

    if (!hitLeft || !hitRight) {
      // Fallback: If clickedPoint provided, use it
      if (clickedPoint) {
        hitLeft = clickedPoint.clone();
        // Project across closing axis to find other side
        const projRay = new THREE.Raycaster(
          clickedPoint.clone().addScaledVector(closingDirWorld, 0.1),
          closingDirWorld,
          0.1,
          100
        );
        const oppHits = projRay.intersectObjects(cadMeshes, false);
        if (oppHits.length > 0) {
          hitRight = oppHits[oppHits.length - 1].point;
        } else {
          hitRight = clickedPoint.clone().addScaledVector(closingDirWorld, 20);
        }
      } else {
        return {
          success: false,
          thickness: 0,
          message: 'Could not detect both outer walls along closing axis'
        };
      }
    }

    const thickness = Number(hitLeft.distanceTo(hitRight).toFixed(1));
    const midpoint = new THREE.Vector3().addVectors(hitLeft, hitRight).multiplyScalar(0.5);

    if (useClickedPointOnOneFinger && clickedPoint) {
      // Place clicked finger pad on the clicked surface point
      // Vector from midpoint to clicked point
      const deltaX = clickedPoint.clone().sub(tcpPos).dot(closingDirWorld);
      gripperModel.root.position.addScaledVector(closingDirWorld, deltaX);
    } else {
      // Move TCP to midpoint along the closing axis
      const currentProj = tcpPos.clone().dot(closingDirWorld);
      const targetProj = midpoint.clone().dot(closingDirWorld);
      const shift = targetProj - currentProj;
      gripperModel.root.position.addScaledVector(closingDirWorld, shift);
    }

    gripperModel.root.updateMatrixWorld(true);

    // Open fingers enough to clear thickness + 10 mm
    const recommendedOpening = Math.min(
      GRIPPER_CONFIG.maxOpeningWidth,
      Math.max(thickness + 10, 30)
    );
    gripperModel.setOpeningWidth(recommendedOpening);
    gripperModel.root.updateMatrixWorld(true);

    return {
      success: true,
      thickness,
      midpointWorld: midpoint,
      message: `Centered on part: thickness = ${thickness} mm`
    };
  }

  /**
   * 2c. Snap to Clicked Point with Standoff and Depth:
   * Positions the gripper so the clicked point is the contact point on the part wall,
   * with approach pointing toward the part (Z = -normal), plus standoff and depth.
   */
  public static snapToContact(
    point: THREE.Vector3,
    normal: THREE.Vector3,
    standoffMm = 0,
    depthMm = 15,
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null
  ) {
    // Approach vector pointing into part surface: Z = -normal
    const approachDir = normal.clone().negate().normalize();
    const defaultZ = new THREE.Vector3(0, 0, 1);

    const quat = new THREE.Quaternion();
    if (approachDir.distanceTo(new THREE.Vector3(0, 0, -1)) < 0.0001) {
      quat.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
    } else {
      quat.setFromUnitVectors(defaultZ, approachDir);
    }
    gripperModel.root.quaternion.copy(quat);

    // Target position: at surface point, moved forward along approach by depth,
    // and backed off by standoff along -approach
    const netApproachShift = depthMm - standoffMm;
    const targetPos = point.clone().addScaledVector(approachDir, netApproachShift);

    gripperModel.root.position.copy(targetPos);
    gripperModel.root.updateMatrixWorld(true);
  }

  private static extractPartFrameContacts(
    state: { leftContactWorld?: THREE.Vector3; rightContactWorld?: THREE.Vector3 },
    cadPartsRoot: THREE.Group | null
  ): ContactPoints {
    if (!cadPartsRoot) return {};
    const res: ContactPoints = {};
    if (state.leftContactWorld) {
      res.left = PoseMath.worldPointToPartFrame(state.leftContactWorld, cadPartsRoot.matrixWorld);
    }
    if (state.rightContactWorld) {
      res.right = PoseMath.worldPointToPartFrame(state.rightContactWorld, cadPartsRoot.matrixWorld);
    }
    return res;
  }
}
