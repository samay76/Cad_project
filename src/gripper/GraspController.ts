import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GripperModel } from './GripperModel';
import { CollisionChecker, type DetailedCollisionResult } from './collision';
import { ContactGrip, type CloseUntilContactResult, type AutoPlaceResult } from './ContactGrip';
import { PoseMath } from '../math/poses';
import { PartController } from '../part/PartController';
import { GRIPPER_CONFIG } from '../config';
import type {
  Pose,
  EulerAngles,
  GripperState,
  TestPhase,
  ContactStatusType,
  ContactPoints,
  Vector3D
} from '../types';

export type GripperStateCallback = (state: GripperState) => void;

export class GraspController {
  private gripperModel: GripperModel;
  private transformControls: TransformControls;
  private orbitControls: OrbitControls;
  private scene: THREE.Scene;
  private cadPartsRoot: THREE.Group | null = null;
  private partController: PartController | null = null;

  // Active edit target: 'part' or 'gripper'
  private activeEditTarget: 'part' | 'gripper' = 'gripper';

  // Configurable contact & grasp settings
  private standoff = 0; // mm
  private depth = 15; // mm
  private tolerance = 0.1; // mm
  private preGraspDistance = GRIPPER_CONFIG.defaultPreGraspDistance;

  // Contact status state
  private contactStatus: ContactStatusType = 'none';
  private contactMessage = 'No collision';
  private leftFingerContact = false;
  private rightFingerContact = false;
  private measuredGraspWidth: number | undefined;
  private lastDetectedContactWidth = 0;
  private contactPointsPart: ContactPoints = {};

  private onStateChange: GripperStateCallback | null = null;

  // Animation test sequence state
  private isTesting = false;
  private testPhase: TestPhase = 'idle';
  private animationId: number | null = null;
  private savedPoseBeforeTest: {
    position: THREE.Vector3;
    quaternion: THREE.Quaternion;
    openingWidth: number;
  } | null = null;
  private savedCadPosBeforeTest: THREE.Vector3 | null = null;
  private savedCadQuatBeforeTest: THREE.Quaternion | null = null;

  // Keyboard shortcut listener
  private boundKeyDown: (e: KeyboardEvent) => void;

  constructor(
    gripperModel: GripperModel,
    camera: THREE.Camera,
    domElement: HTMLElement,
    scene: THREE.Scene,
    orbitControls: OrbitControls,
    partController?: PartController
  ) {
    this.gripperModel = gripperModel;
    this.scene = scene;
    this.orbitControls = orbitControls;
    this.partController = partController || null;

    // Attach gripper model root to scene
    this.scene.add(this.gripperModel.root);

    // Initialize TransformControls
    this.transformControls = new TransformControls(camera, domElement);
    this.transformControls.size = 0.85;
    this.transformControls.setSpace('local');
    this.transformControls.attach(this.gripperModel.root);
    this.scene.add(this.transformControls.getHelper());

    // Prevent OrbitControls conflict when dragging gizmo
    this.transformControls.addEventListener('dragging-changed', (event) => {
      this.orbitControls.enabled = !event.value;
    });

    this.transformControls.addEventListener('change', () => {
      if (this.activeEditTarget === 'part') {
        this.partController?.handlePartMoved();
        this.evaluateCollisionState();
      } else {
        this.handleGizmoMove();
      }
    });

    // Keyboard shortcuts: T for Translate, R for Rotate
    this.boundKeyDown = (e: KeyboardEvent) => {
      if (['INPUT', 'TEXTAREA', 'SELECT'].includes((e.target as HTMLElement)?.tagName)) {
        return;
      }
      if (e.key === 't' || e.key === 'T') {
        this.setTransformMode('translate');
      } else if (e.key === 'r' || e.key === 'R') {
        this.setTransformMode('rotate');
      }
    };
    window.addEventListener('keydown', this.boundKeyDown);

    if (this.partController) {
      this.partController.setGripperRoot(this.gripperModel.root);
    }

    // Initial default pose
    this.resetGripperPose();
  }

  public setPartController(controller: PartController) {
    this.partController = controller;
    this.partController.setGripperRoot(this.gripperModel.root);
    if (this.cadPartsRoot) {
      this.partController.setCadPartsRoot(this.cadPartsRoot);
    }
  }

  public setCadPartsRoot(root: THREE.Group | null) {
    this.cadPartsRoot = root;
    if (this.partController) {
      this.partController.setCadPartsRoot(root);
    }

    // Reattach gizmo if part was active target
    if (this.activeEditTarget === 'part') {
      if (this.cadPartsRoot) {
        this.transformControls.detach();
        this.transformControls.attach(this.cadPartsRoot);
      } else {
        this.setActiveEditTarget('gripper');
      }
    }

    this.evaluateCollisionState();
    this.notifyState();
  }

  public setOnStateChange(callback: GripperStateCallback) {
    this.onStateChange = callback;
    this.notifyState();
  }

  /**
   * Toggle between editing Part and Gripper
   */
  public setActiveEditTarget(target: 'part' | 'gripper') {
    this.activeEditTarget = target;

    this.transformControls.detach();
    if (target === 'part' && this.cadPartsRoot) {
      this.transformControls.attach(this.cadPartsRoot);
    } else {
      this.transformControls.attach(this.gripperModel.root);
    }

    this.notifyState();
  }

  public getActiveEditTarget(): 'part' | 'gripper' {
    return this.activeEditTarget;
  }

  public setTransformMode(mode: 'translate' | 'rotate') {
    this.transformControls.setMode(mode);
    this.notifyState();
  }

  public getTransformMode(): 'translate' | 'rotate' {
    return this.transformControls.getMode() as 'translate' | 'rotate';
  }

  public setTransformVisible(visible: boolean) {
    this.transformControls.getHelper().visible = visible;
    this.transformControls.enabled = visible;
  }

  private handleGizmoMove() {
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * Live BVH collision & contact evaluation
   */
  public evaluateCollisionState(): DetailedCollisionResult {
    const res = CollisionChecker.evaluateState(
      this.gripperModel,
      this.cadPartsRoot,
      this.tolerance
    );

    // Highlight penetrating meshes in red
    this.gripperModel.highlightMeshes(res.penetratingMeshes);

    this.contactStatus = res.status;
    this.contactMessage = res.message;
    this.leftFingerContact = res.leftPadContact;
    this.rightFingerContact = res.rightPadContact;

    if (res.leftPadContact && res.rightPadContact && res.leftContactWorld && res.rightContactWorld) {
      this.lastDetectedContactWidth = Number(
        res.leftContactWorld.distanceTo(res.rightContactWorld).toFixed(1)
      );
    }

    return res;
  }

  /**
   * 2b. Close fingers until contact with outer wall (no penetration)
   */
  public closeUntilContact(tolerance?: number): CloseUntilContactResult {
    if (tolerance !== undefined) {
      this.tolerance = tolerance;
    }

    const result = ContactGrip.closeUntilContact(
      this.gripperModel,
      this.cadPartsRoot,
      this.tolerance
    );

    this.contactStatus = result.status;
    this.contactMessage = result.message;
    this.measuredGraspWidth = result.measuredGraspWidth;
    this.leftFingerContact = result.leftContact;
    this.rightFingerContact = result.rightContact;
    this.contactPointsPart = result.contactPointsPart;

    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();

    return result;
  }

  /**
   * 2c. Open to clear the CAD part
   */
  public openToClear(): { cleared: boolean; message: string } {
    const res = ContactGrip.openToClear(this.gripperModel, this.cadPartsRoot);
    this.contactMessage = res.message;
    if (!res.cleared) {
      this.contactStatus = 'collision';
    } else {
      this.contactStatus = 'none';
      this.measuredGraspWidth = undefined;
    }

    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    return res;
  }

  /**
   * 2c. Auto-place on outer wall:
   * Casts rays along closing axis to find outer wall hits and centers TCP.
   */
  public autoPlaceOnOuterWall(
    clickedPoint?: THREE.Vector3,
    useClickedPointOnOneFinger = false
  ): AutoPlaceResult {
    const res = ContactGrip.autoPlaceOnOuterWall(
      this.gripperModel,
      this.cadPartsRoot,
      clickedPoint,
      useClickedPointOnOneFinger
    );

    this.contactMessage = res.message;
    if (res.success) {
      this.contactStatus = 'contact';
      this.lastDetectedContactWidth = res.thickness;
    }

    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    return res;
  }

  /**
   * 2c. Snap to Clicked Point with Standoff and Depth
   */
  public snapToClickedPoint(
    point: { x: number; y: number; z: number },
    normal: { x: number; y: number; z: number },
    standoffMm = this.standoff,
    depthMm = this.depth
  ) {
    this.standoff = standoffMm;
    this.depth = depthMm;

    ContactGrip.snapToContact(
      new THREE.Vector3(point.x, point.y, point.z),
      new THREE.Vector3(normal.x, normal.y, normal.z),
      this.standoff,
      this.depth,
      this.gripperModel,
      this.cadPartsRoot
    );

    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * Flips approach direction 180 degrees
   */
  public flipApproach() {
    this.gripperModel.root.rotateX(Math.PI);
    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * Rotates around approach axis (+Z) by degrees
   */
  public rotateAroundApproach(degrees: number) {
    const rad = THREE.MathUtils.degToRad(degrees);
    this.gripperModel.root.rotateZ(rad);
    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * Translates gripper along axis by delta in mm
   */
  public stepTranslate(axis: 'x' | 'y' | 'z', deltaMm: number) {
    this.gripperModel.root.position[axis] += deltaMm;
    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * Rotates gripper around local axis by delta in degrees
   */
  public stepRotate(axis: 'x' | 'y' | 'z', deltaDeg: number) {
    const rad = THREE.MathUtils.degToRad(deltaDeg);
    if (axis === 'x') this.gripperModel.root.rotateX(rad);
    if (axis === 'y') this.gripperModel.root.rotateY(rad);
    if (axis === 'z') this.gripperModel.root.rotateZ(rad);
    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * Sets numeric TCP pose in part coordinate frame
   */
  public setNumericPose(pos: { x: number; y: number; z: number }, rpy: EulerAngles) {
    if (this.cadPartsRoot) {
      // Input is T_part_grasp: compute world pose T_world_grasp = T_world_part * T_part_grasp
      const graspPose: Pose = {
        position: pos,
        orientation: { x: 0, y: 0, z: 0, w: 1 }
      };
      const quat = PoseMath.eulerRPYToQuaternion(rpy);
      graspPose.orientation = { x: quat.x, y: quat.y, z: quat.z, w: quat.w };

      const world = PoseMath.computeWorldGraspTransform(this.cadPartsRoot.matrixWorld, graspPose);
      this.gripperModel.root.position.copy(world.position);
      this.gripperModel.root.quaternion.copy(world.quaternion);
    } else {
      this.gripperModel.root.position.set(pos.x, pos.y, pos.z);
      this.gripperModel.root.quaternion.copy(PoseMath.eulerRPYToQuaternion(rpy));
    }

    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  public setOpeningWidth(widthMm: number) {
    this.gripperModel.setOpeningWidth(widthMm);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  public setTcpOffset(offsetMm: number) {
    this.gripperModel.setTcpOffset(offsetMm);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  public setStandoff(standoffMm: number) {
    this.standoff = standoffMm;
    this.notifyState();
  }

  public setDepth(depthMm: number) {
    this.depth = depthMm;
    this.notifyState();
  }

  public setTolerance(tolMm: number) {
    this.tolerance = Math.max(0.01, tolMm);
    this.evaluateCollisionState();
    this.notifyState();
  }

  public setPreGraspDistance(distMm: number) {
    this.preGraspDistance = Math.max(10, distMm);
    this.notifyState();
  }

  public resetGripperPose() {
    if (this.cadPartsRoot) {
      const box = new THREE.Box3().setFromObject(this.cadPartsRoot);
      const center = box.getCenter(new THREE.Vector3());
      this.gripperModel.root.position.set(center.x, center.y, box.max.z + 50);
      this.gripperModel.root.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
    } else {
      this.gripperModel.root.position.set(0, 0, 100);
      this.gripperModel.root.quaternion.set(0, 0, 0, 1);
    }

    this.gripperModel.setOpeningWidth(GRIPPER_CONFIG.defaultOpeningWidth);
    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * 2d. Updated Test Grasp sequence:
   * Pre-grasp (open, outside the part) ->
   * Approach (move straight along approach Z while checking collisions; stop and show red if collision) ->
   * Close with contact detection (2b) ->
   * Lift (CAD model moves together with gripper!) ->
   * Transfer -> Place -> Release -> Retract -> Restore.
   */
  public testGrasp() {
    if (this.isTesting) return;

    this.isTesting = true;
    this.transformControls.enabled = false;
    this.transformControls.getHelper().visible = false;

    // Save initial editable grasp pose and CAD model initial position
    this.savedPoseBeforeTest = {
      position: this.gripperModel.root.position.clone(),
      quaternion: this.gripperModel.root.quaternion.clone(),
      openingWidth: this.gripperModel.getOpeningWidth()
    };

    if (this.cadPartsRoot) {
      this.savedCadPosBeforeTest = this.cadPartsRoot.position.clone();
      this.savedCadQuatBeforeTest = this.cadPartsRoot.quaternion.clone();
    }

    const graspPos = this.gripperModel.root.position.clone();
    const graspQuat = this.gripperModel.root.quaternion.clone();

    // Approach vector in world coordinates (+Z of TCP)
    const approachVec = new THREE.Vector3(0, 0, 1).applyQuaternion(graspQuat).normalize();
    const preGraspPos = graspPos.clone().addScaledVector(approachVec, -this.preGraspDistance);

    // Pick & Lift target: lifts 80mm vertically along world Z
    const liftOffset = new THREE.Vector3(0, 0, 80);
    const liftPos = graspPos.clone().add(liftOffset);

    // Place location
    const transferOffset = new THREE.Vector3(70, 0, 80);
    const transferPos = graspPos.clone().add(transferOffset);
    const placePos = graspPos.clone().add(new THREE.Vector3(70, 0, 0));

    let clampedTargetWidth = this.gripperModel.getOpeningWidth();
    let isGraspAttached = false;

    const startTime = performance.now();
    const phaseDurations = {
      preGrasp: 700,
      approach: 900,
      close: 800,
      lift: 900,
      transfer: 800,
      place: 700,
      release: 500,
      retract: 700
    };

    const tPreGrasp = phaseDurations.preGrasp;
    const tApproach = tPreGrasp + phaseDurations.approach;
    const tClose = tApproach + phaseDurations.close;
    const tLift = tClose + phaseDurations.lift;
    const tTransfer = tLift + phaseDurations.transfer;
    const tPlace = tTransfer + phaseDurations.place;
    const tRelease = tPlace + phaseDurations.release;
    const totalDuration = tRelease + phaseDurations.retract;

    const animateStep = (now: number) => {
      if (!this.isTesting) return;

      const elapsed = now - startTime;

      if (elapsed < tPreGrasp) {
        // Phase 1: Pre-Grasp (open fingers and move to pre-grasp standoff)
        this.testPhase = 'pre-grasp';
        const t = Math.min(elapsed / phaseDurations.preGrasp, 1);
        const ease = 1 - Math.pow(1 - t, 2);
        this.gripperModel.root.position.lerpVectors(graspPos, preGraspPos, ease);
        this.gripperModel.setOpeningWidth(
          THREE.MathUtils.lerp(this.savedPoseBeforeTest!.openingWidth, GRIPPER_CONFIG.maxOpeningWidth, ease)
        );
      } else if (elapsed < tApproach) {
        // Phase 2: Approach (move straight along approach Z while checking collisions)
        this.testPhase = 'approach';
        const t = Math.min((elapsed - tPreGrasp) / phaseDurations.approach, 1);
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(preGraspPos, graspPos, ease);
        this.gripperModel.setOpeningWidth(GRIPPER_CONFIG.maxOpeningWidth);

        // Check for premature collision during approach
        this.gripperModel.root.updateMatrixWorld(true);
        const colCheck = CollisionChecker.evaluateState(this.gripperModel, this.cadPartsRoot, this.tolerance);
        if (colCheck.hasCollision && t < 0.9) {
          // Stop and show red if open gripper hits part before reaching grasp depth
          this.contactStatus = 'collision';
          this.contactMessage = 'Collision during approach: open gripper hit CAD part!';
          this.gripperModel.highlightMeshes(colCheck.penetratingMeshes);
          this.notifyState();
          setTimeout(() => this.stopTestGrasp(), 1800);
          return;
        }
      } else if (elapsed < tClose) {
        // Phase 3: Close (close with contact detection 2b)
        if (this.testPhase !== 'close') {
          this.testPhase = 'close';
          this.gripperModel.root.position.copy(graspPos);
          this.gripperModel.root.updateMatrixWorld(true);

          // Run contact-aware closing
          const closeRes = ContactGrip.closeUntilContact(
            this.gripperModel,
            this.cadPartsRoot,
            this.tolerance
          );
          clampedTargetWidth = closeRes.measuredGraspWidth;
          this.measuredGraspWidth = clampedTargetWidth;
          this.contactStatus = closeRes.status;
          this.contactMessage = closeRes.message;
          this.leftFingerContact = closeRes.leftContact;
          this.rightFingerContact = closeRes.rightContact;
          this.contactPointsPart = closeRes.contactPointsPart;

          isGraspAttached = closeRes.status === 'contact';
        }
      } else if (elapsed < tLift) {
        // Phase 4: Lift (move CAD model together with gripper to visually confirm grip)
        this.testPhase = 'lift';
        const t = Math.min((elapsed - tClose) / phaseDurations.lift, 1);
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(graspPos, liftPos, ease);
        this.gripperModel.setOpeningWidth(clampedTargetWidth);

        if (isGraspAttached && this.cadPartsRoot && this.savedCadPosBeforeTest) {
          const delta = this.gripperModel.root.position.clone().sub(graspPos);
          this.cadPartsRoot.position.copy(this.savedCadPosBeforeTest).add(delta);
          this.cadPartsRoot.updateMatrixWorld(true);
        }
      } else if (elapsed < tTransfer) {
        // Phase 5: Transfer (carry CAD model horizontally)
        this.testPhase = 'transfer';
        const t = Math.min((elapsed - tLift) / phaseDurations.transfer, 1);
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(liftPos, transferPos, ease);
        this.gripperModel.setOpeningWidth(clampedTargetWidth);

        if (isGraspAttached && this.cadPartsRoot && this.savedCadPosBeforeTest) {
          const delta = this.gripperModel.root.position.clone().sub(graspPos);
          this.cadPartsRoot.position.copy(this.savedCadPosBeforeTest).add(delta);
          this.cadPartsRoot.updateMatrixWorld(true);
        }
      } else if (elapsed < tPlace) {
        // Phase 6: Place (lower CAD model onto resting surface)
        this.testPhase = 'place';
        const t = Math.min((elapsed - tTransfer) / phaseDurations.place, 1);
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(transferPos, placePos, ease);
        this.gripperModel.setOpeningWidth(clampedTargetWidth);

        if (isGraspAttached && this.cadPartsRoot && this.savedCadPosBeforeTest) {
          const delta = this.gripperModel.root.position.clone().sub(graspPos);
          this.cadPartsRoot.position.copy(this.savedCadPosBeforeTest).add(delta);
          this.cadPartsRoot.updateMatrixWorld(true);
        }
      } else if (elapsed < tRelease) {
        // Phase 7: Release (open fingers, releasing CAD model)
        this.testPhase = 'release';
        const t = Math.min((elapsed - tPlace) / phaseDurations.release, 1);
        const ease = 1 - Math.pow(1 - t, 2);
        this.gripperModel.root.position.copy(placePos);
        this.gripperModel.setOpeningWidth(
          THREE.MathUtils.lerp(clampedTargetWidth, GRIPPER_CONFIG.maxOpeningWidth, ease)
        );
      } else if (elapsed < totalDuration) {
        // Phase 8: Retract (lift gripper alone back to clearance)
        this.testPhase = 'retract';
        const t = Math.min((elapsed - tRelease) / phaseDurations.retract, 1);
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(placePos, transferPos, ease);
        this.gripperModel.setOpeningWidth(GRIPPER_CONFIG.maxOpeningWidth);
      } else {
        setTimeout(() => {
          this.stopTestGrasp();
        }, 1200);
        return;
      }

      this.gripperModel.root.updateMatrixWorld(true);
      this.notifyState();
      this.animationId = requestAnimationFrame(animateStep);
    };

    this.animationId = requestAnimationFrame(animateStep);
  }

  public stopTestGrasp() {
    if (this.animationId !== null) {
      cancelAnimationFrame(this.animationId);
      this.animationId = null;
    }

    this.isTesting = false;
    this.testPhase = 'idle';

    // Restore editable grasp pose
    if (this.savedPoseBeforeTest) {
      this.gripperModel.root.position.copy(this.savedPoseBeforeTest.position);
      this.gripperModel.root.quaternion.copy(this.savedPoseBeforeTest.quaternion);
      this.gripperModel.setOpeningWidth(this.savedPoseBeforeTest.openingWidth);
      this.gripperModel.root.updateMatrixWorld(true);
    }

    // Restore CAD model position
    if (this.cadPartsRoot && this.savedCadPosBeforeTest) {
      this.cadPartsRoot.position.copy(this.savedCadPosBeforeTest);
      if (this.savedCadQuatBeforeTest) {
        this.cadPartsRoot.quaternion.copy(this.savedCadQuatBeforeTest);
      }
      this.cadPartsRoot.updateMatrixWorld(true);
    }

    this.transformControls.enabled = true;
    this.transformControls.getHelper().visible = true;

    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * Computes current grasp state.
   * Saved grasps are always expressed as T_part_grasp relative to CAD part frame!
   */
  public getCurrentState(): GripperState {
    let partGraspPose: Pose;
    let rpy: EulerAngles;

    if (this.cadPartsRoot) {
      partGraspPose = PoseMath.computePartGraspPose(
        this.cadPartsRoot.matrixWorld,
        this.gripperModel.root.matrixWorld
      );
      const quat = new THREE.Quaternion(
        partGraspPose.orientation.x,
        partGraspPose.orientation.y,
        partGraspPose.orientation.z,
        partGraspPose.orientation.w
      );
      rpy = PoseMath.quaternionToEulerRPY(quat);
    } else {
      const pos = this.gripperModel.root.position;
      const quat = this.gripperModel.root.quaternion;
      partGraspPose = {
        position: {
          x: Number(pos.x.toFixed(3)),
          y: Number(pos.y.toFixed(3)),
          z: Number(pos.z.toFixed(3))
        },
        orientation: {
          x: Number(quat.x.toFixed(4)),
          y: Number(quat.y.toFixed(4)),
          z: Number(quat.z.toFixed(4)),
          w: Number(quat.w.toFixed(4))
        }
      };
      rpy = PoseMath.quaternionToEulerRPY(quat);
    }

    return {
      loaded: true,
      visible: this.gripperModel.isVisible(),
      openingWidth: Number(this.gripperModel.getOpeningWidth().toFixed(1)),
      measuredGraspWidth: this.measuredGraspWidth,
      contactWidth:
        this.lastDetectedContactWidth > 0
          ? Number(this.lastDetectedContactWidth.toFixed(1))
          : undefined,
      contactStatus: this.contactStatus,
      contactMessage: this.contactMessage,
      leftFingerContact: this.leftFingerContact,
      rightFingerContact: this.rightFingerContact,
      standoff: this.standoff,
      depth: this.depth,
      tcpOffset: Number(this.gripperModel.getTcpOffset().toFixed(1)),
      preGraspDistance: Number(this.preGraspDistance.toFixed(1)),
      pose: partGraspPose,
      rpy,
      hasCollision: this.gripperModel.getIsColliding(),
      isTesting: this.isTesting,
      testPhase: this.testPhase,
      activeEditTarget: this.activeEditTarget
    };
  }

  public getContactPointsPart(): ContactPoints {
    return this.contactPointsPart;
  }

  public getGripperModel(): GripperModel {
    return this.gripperModel;
  }

  private notifyState() {
    if (this.onStateChange) {
      this.onStateChange(this.getCurrentState());
    }
  }

  public dispose() {
    if (this.animationId !== null) {
      cancelAnimationFrame(this.animationId);
    }
    window.removeEventListener('keydown', this.boundKeyDown);
    this.transformControls.dispose();
    this.gripperModel.dispose();
  }
}
