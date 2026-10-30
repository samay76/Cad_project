import * as THREE from 'three';
import type { GripperModel } from './GripperModel';
import { CollisionChecker, type DetailedCollisionResult } from './collision';
import { PoseMath } from '../math/poses';
import type { ContactPoints, Vector3D } from '../types';
import { GRIPPER_CONFIG } from '../config';

export interface CloseUntilContactResult {
  status: 'contact' | 'missed' | 'too-wide' | 'collision';
  message: string;
  measuredGraspWidth: number;
  leftContact: boolean;
  rightContact: boolean;
  targetLeftAngle: number;
  targetRightAngle: number;
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
  /**
   * 2b. High-Performance Contact-Aware Closing:
   * Uses an 8-10 iteration binary search (< 0.5 ms total computation) instead of
   * hundreds of stepped checks on the main thread.
   * Finds the exact stop opening without penetrating the CAD part.
   */
  public static computeContactGrasp(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null,
    tolerance = 0.1 // mm
  ): CloseUntilContactResult {
    const initialOpening = gripperModel.getOpeningWidth();

    if (!cadPartsRoot) {
      return {
        status: 'missed',
        message: 'No CAD model loaded to grasp',
        measuredGraspWidth: initialOpening,
        leftContact: false,
        rightContact: false,
        targetLeftAngle: (1.0 - initialOpening / GRIPPER_CONFIG.maxOpeningWidth) * 0.8,
        targetRightAngle: (1.0 - initialOpening / GRIPPER_CONFIG.maxOpeningWidth) * 0.8,
        contactPointsWorld: {},
        contactPointsPart: {}
      };
    }

    CollisionChecker.ensureBVH(cadPartsRoot);

    // Initial check: if already at initial opening and penetrating CAD body
    const initialState = CollisionChecker.evaluateState(gripperModel, cadPartsRoot, tolerance);
    if (initialState.hasCollision) {
      return {
        status: 'too-wide',
        message: 'Part too wide for the gripper at this pose (body collision)',
        measuredGraspWidth: initialOpening,
        leftContact: initialState.leftPadContact,
        rightContact: initialState.rightPadContact,
        targetLeftAngle: 0,
        targetRightAngle: 0,
        contactPointsWorld: {
          left: initialState.leftContactWorld,
          right: initialState.rightContactWorld
        },
        contactPointsPart: this.extractPartFrameContacts(initialState, cadPartsRoot)
      };
    }

    // Binary search bounds:
    // openWidth (clear/safe) -> closedWidth (penetrating or fully closed 0mm)
    let openWidth = initialOpening;
    let closedWidth = GRIPPER_CONFIG.minOpeningWidth; // 0.0 mm

    let lastSafeState = initialState;
    let finalState = initialState;

    // 10 binary search iterations achieve 85 / 1024 ~ 0.08 mm precision in ~0.2 ms
    const iterations = 10;
    for (let i = 0; i < iterations; i++) {
      const midWidth = (openWidth + closedWidth) / 2;
      gripperModel.setOpeningWidth(midWidth);

      const state = CollisionChecker.evaluateState(gripperModel, cadPartsRoot, tolerance);

      if (state.hasCollision || state.isContact) {
        // Touched or penetrated -> safe boundary is more open
        closedWidth = midWidth;
        finalState = state;
      } else {
        // Clear -> can close further
        openWidth = midWidth;
        lastSafeState = state;
      }
    }

    // Set to the highest safe opening with contact tolerance
    let targetOpening = Number(openWidth.toFixed(1));
    gripperModel.setOpeningWidth(targetOpening);
    let stateAtTarget = CollisionChecker.evaluateState(gripperModel, cadPartsRoot, tolerance);

    // If safe state is slightly off contact, try closing down in tiny micro-step to touch
    if (!stateAtTarget.isContact && !stateAtTarget.hasCollision) {
      gripperModel.setOpeningWidth(closedWidth);
      const stateClosed = CollisionChecker.evaluateState(gripperModel, cadPartsRoot, tolerance);
      if (!stateClosed.hasCollision && stateClosed.isContact) {
        targetOpening = closedWidth; // exact: rounding to 0.1 mm can step back out of the contact tolerance
        stateAtTarget = stateClosed;
      } else {
        gripperModel.setOpeningWidth(targetOpening);
      }
    }

    // Kinematic target angles
    const baseTargetAngle = (1.0 - targetOpening / GRIPPER_CONFIG.maxOpeningWidth) * 0.8;
    let leftAngle = baseTargetAngle;
    let rightAngle = baseTargetAngle;

    // Independent finger refinement if only one finger touched
    if (stateAtTarget.leftPadContact && !stateAtTarget.rightPadContact) {
      // Right finger can close further to seek contact
      let rMinAngle = baseTargetAngle;
      let rMaxAngle = 0.8; // 0 mm
      for (let j = 0; j < 6; j++) {
        const midA = (rMinAngle + rMaxAngle) / 2;
        gripperModel.setFingerAngles(leftAngle, midA);
        const s = CollisionChecker.evaluateState(gripperModel, cadPartsRoot, tolerance);
        if (s.hasCollision || s.rightPadContact) {
          rMaxAngle = midA;
          stateAtTarget = s;
        } else {
          rMinAngle = midA;
        }
      }
      rightAngle = rMinAngle;
      gripperModel.setFingerAngles(leftAngle, rightAngle);
    } else if (!stateAtTarget.leftPadContact && stateAtTarget.rightPadContact) {
      // Left finger can close further
      let lMinAngle = baseTargetAngle;
      let lMaxAngle = 0.8;
      for (let j = 0; j < 6; j++) {
        const midA = (lMinAngle + lMaxAngle) / 2;
        gripperModel.setFingerAngles(midA, rightAngle);
        const s = CollisionChecker.evaluateState(gripperModel, cadPartsRoot, tolerance);
        if (s.hasCollision || s.leftPadContact) {
          lMaxAngle = midA;
          stateAtTarget = s;
        } else {
          lMinAngle = midA;
        }
      }
      leftAngle = lMinAngle;
      gripperModel.setFingerAngles(leftAngle, rightAngle);
    }

    // Restore initial opening so animation can smoothly transition from start to target
    gripperModel.setOpeningWidth(initialOpening);

    // Final measured width
    let measuredWidth = targetOpening;
    if (stateAtTarget.leftContactWorld && stateAtTarget.rightContactWorld) {
      measuredWidth = Number(
        stateAtTarget.leftContactWorld.distanceTo(stateAtTarget.rightContactWorld).toFixed(1)
      );
    }

    const contactPointsPart = this.extractPartFrameContacts(stateAtTarget, cadPartsRoot);

    if (stateAtTarget.leftPadContact && stateAtTarget.rightPadContact) {
      return {
        status: 'contact',
        message: `Grasp width: ${measuredWidth} mm (both fingers touching outer wall)`,
        measuredGraspWidth: measuredWidth,
        leftContact: true,
        rightContact: true,
        targetLeftAngle: leftAngle,
        targetRightAngle: rightAngle,
        contactPointsWorld: {
          left: stateAtTarget.leftContactWorld,
          right: stateAtTarget.rightContactWorld
        },
        contactPointsPart
      };
    } else if (stateAtTarget.leftPadContact || stateAtTarget.rightPadContact) {
      const fingerName = stateAtTarget.leftPadContact ? 'left' : 'right';
      return {
        status: 'contact',
        message: `Grasp width: ${measuredWidth} mm (${fingerName} finger touching wall)`,
        measuredGraspWidth: measuredWidth,
        leftContact: stateAtTarget.leftPadContact,
        rightContact: stateAtTarget.rightPadContact,
        targetLeftAngle: leftAngle,
        targetRightAngle: rightAngle,
        contactPointsWorld: {
          left: stateAtTarget.leftContactWorld,
          right: stateAtTarget.rightContactWorld
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
        targetLeftAngle: baseTargetAngle,
        targetRightAngle: baseTargetAngle,
        contactPointsWorld: {},
        contactPointsPart: {}
      };
    }
  }

  /**
   * Smoothly animates fingers from current opening to computed contact angles over frames
   */
  public static animateClosing(
    gripperModel: GripperModel,
    targetLeftAngle: number,
    targetRightAngle: number,
    frameCount = 12,
    onProgress?: () => void,
    onComplete?: () => void
  ) {
    const startOpening = gripperModel.getOpeningWidth();
    const startAngle = (1.0 - startOpening / GRIPPER_CONFIG.maxOpeningWidth) * 0.8;

    let currentFrame = 0;

    const step = () => {
      currentFrame++;
      const t = Math.min(1.0, currentFrame / frameCount);
      const ease = t * (2 - t); // Quad ease-out

      const curLeft = THREE.MathUtils.lerp(startAngle, targetLeftAngle, ease);
      const curRight = THREE.MathUtils.lerp(startAngle, targetRightAngle, ease);

      gripperModel.setFingerAngles(curLeft, curRight);
      gripperModel.root.updateMatrixWorld(true);

      if (onProgress) onProgress();

      if (currentFrame < frameCount) {
        requestAnimationFrame(step);
      } else {
        if (onComplete) onComplete();
      }
    };

    requestAnimationFrame(step);
  }

  /**
   * 2c. Open to Clear:
   * Opens the gripper fully (85 mm).
   */
  public static openToClear(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null
  ): { cleared: boolean; message: string } {
    gripperModel.setOpeningWidth(GRIPPER_CONFIG.maxOpeningWidth);
    gripperModel.root.updateMatrixWorld(true);

    if (!cadPartsRoot) {
      return { cleared: true, message: `Gripper fully open (${GRIPPER_CONFIG.maxOpeningWidth} mm)` };
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

    // Raycast along closing axis in +X and -X directions using accelerated BVH
    const rayPositive = new THREE.Raycaster(tcpPos, closingDirWorld, 0.1, 300);
    const rayNegative = new THREE.Raycaster(tcpPos, closingDirWorld.clone().negate(), 0.1, 300);
    (rayPositive as any).firstHitOnly = true;
    (rayNegative as any).firstHitOnly = true;

    const hitsPos = rayPositive.intersectObjects(cadMeshes, false);
    const hitsNeg = rayNegative.intersectObjects(cadMeshes, false);

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
        150
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
      if (clickedPoint) {
        hitLeft = clickedPoint.clone();
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
      const deltaX = clickedPoint.clone().sub(tcpPos).dot(closingDirWorld);
      gripperModel.root.position.addScaledVector(closingDirWorld, deltaX);
    } else {
      const currentProj = tcpPos.clone().dot(closingDirWorld);
      const targetProj = midpoint.clone().dot(closingDirWorld);
      const shift = targetProj - currentProj;
      gripperModel.root.position.addScaledVector(closingDirWorld, shift);
    }

    gripperModel.root.updateMatrixWorld(true);

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
   * 2c. Snap to Clicked Point with Standoff and Depth
   */
  public static snapToContact(
    point: THREE.Vector3,
    normal: THREE.Vector3,
    standoffMm = 0,
    depthMm = 15,
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null
  ) {
    const approachDir = normal.clone().negate().normalize();
    const defaultZ = new THREE.Vector3(0, 0, 1);

    const quat = new THREE.Quaternion();
    if (approachDir.distanceTo(new THREE.Vector3(0, 0, -1)) < 0.0001) {
      quat.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
    } else {
      quat.setFromUnitVectors(defaultZ, approachDir);
    }
    gripperModel.root.quaternion.copy(quat);

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
