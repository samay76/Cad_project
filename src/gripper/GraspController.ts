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

  // Throttling state for gizmo dragging
  private isDraggingGizmo = false;
  private rafColPending = false;
  private lastUiNotifyTime = 0;

  private onStateChange: GripperStateCallback | null = null;
  private onRequestRender: (() => void) | null = null;
  private onDraggingChanged: ((isDragging: boolean) => void) | null = null;

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

  // Scratch objects
  private _scratchQuat = new THREE.Quaternion();

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

    // Attach gripper model root to scene (sibling of partRoot)
    this.scene.add(this.gripperModel.root);

    // Initialize single TransformControls instance
    this.transformControls = new TransformControls(camera, domElement);
    this.transformControls.size = 0.85;
    this.transformControls.setSpace('local');
    this.transformControls.attach(this.gripperModel.root);
    this.scene.add(this.transformControls.getHelper());

    // Prevent OrbitControls conflict when dragging gizmo
    this.transformControls.addEventListener('dragging-changed', (event: any) => {
      const isDragging = !!event.value;
      this.isDraggingGizmo = isDragging;
      this.orbitControls.enabled = !isDragging;

      if (this.onDraggingChanged) {
        this.onDraggingChanged(isDragging);
      }

      if (!isDragging) {
        // Drag end: run full precision collision check and immediate state notification
        this.evaluateCollisionState();
        if (this.partController) {
          this.partController.updateCachedPartGrasp();
        }
        this.notifyState();
      }

      this.requestRender();
    });

    this.transformControls.addEventListener('change', () => {
      if (this.isDraggingGizmo) {
        if (this.activeEditTarget === 'part') {
          this.partController?.handlePartMoved();
        }
        this.scheduleThrottledCollisionCheck();
      } else {
        if (this.activeEditTarget === 'part') {
          this.partController?.handlePartMoved();
          this.evaluateCollisionState();
        } else {
          this.handleGizmoMove();
        }
      }
      this.requestRender();
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

  /**
   * Swaps in another gripper model (gripper selector). The TCP keeps its world pose, so a taught
   * position can be checked with the other gripper; finger opening and TCP offset take the new
   * gripper's defaults.
   */
  public replaceGripperModel(model: GripperModel) {
    if (this.isTesting) this.stopTestGrasp();
    const old = this.gripperModel;
    old.root.updateMatrixWorld(true);
    model.root.position.copy(old.root.position);
    model.root.quaternion.copy(old.root.quaternion);
    model.root.updateMatrixWorld(true);

    this.scene.remove(old.root);
    this.transformControls.detach();
    old.dispose();

    this.gripperModel = model;
    this.scene.add(model.root);
    this.transformControls.attach(
      this.activeEditTarget === 'part' && this.cadPartsRoot ? this.cadPartsRoot : model.root
    );
    if (this.partController) {
      this.partController.setGripperRoot(model.root);
    }

    this.contactStatus = 'none';
    this.contactMessage = 'No collision';
    this.leftFingerContact = false;
    this.rightFingerContact = false;
    this.measuredGraspWidth = undefined;
    this.contactPointsPart = {};

    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
  }

  public getGripperSpec() {
    return this.gripperModel.spec;
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
    this.requestRender();
  }

  public setOnStateChange(callback: GripperStateCallback) {
    this.onStateChange = callback;
    this.notifyState();
  }

  public setOnRequestRender(callback: () => void) {
    this.onRequestRender = callback;
  }

  public setOnDraggingChanged(callback: (isDragging: boolean) => void) {
    this.onDraggingChanged = callback;
  }

  public requestRender() {
    if (this.onRequestRender) {
      this.onRequestRender();
    }
  }

  /**
   * Toggle between editing Part and Gripper with single TransformControls
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
    this.requestRender();
  }

  public getActiveEditTarget(): 'part' | 'gripper' {
    return this.activeEditTarget;
  }

  public setTransformMode(mode: 'translate' | 'rotate') {
    this.transformControls.setMode(mode);
    this.notifyState();
    this.requestRender();
  }

  public getTransformMode(): 'translate' | 'rotate' {
    return this.transformControls.getMode() as 'translate' | 'rotate';
  }

  public setTransformVisible(visible: boolean) {
    this.transformControls.getHelper().visible = visible;
    this.transformControls.enabled = visible;
    this.requestRender();
  }

  private handleGizmoMove() {
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * Throttles collision checks to at most 1 per animation frame while dragging
   */
  private scheduleThrottledCollisionCheck() {
    if (this.rafColPending) return;
    this.rafColPending = true;
    requestAnimationFrame(() => {
      this.rafColPending = false;
      this.evaluateCollisionState();
      this.throttledNotifyState();
      this.requestRender();
    });
  }

  /**
   * Throttles UI text/input updates to <= 10 Hz while dragging
   */
  private throttledNotifyState() {
    const now = performance.now();
    if (now - this.lastUiNotifyTime >= 100) {
      this.lastUiNotifyTime = now;
      this.notifyState();
    }
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

    // Highlight penetrating visual meshes in red
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
   * 2b. High-Performance Contact Closing:
   * Binary search computation (< 0.5 ms) followed by smooth animation over frames
   */
  public closeUntilContact(tolerance?: number): CloseUntilContactResult {
    if (tolerance !== undefined) {
      this.tolerance = tolerance;
    }

    const result = ContactGrip.computeContactGrasp(
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

    // Smoothly animate closing to computed target angles
    ContactGrip.animateClosing(
      this.gripperModel,
      result.targetLeftAngle,
      result.targetRightAngle,
      12,
      () => {
        this.requestRender();
      },
      () => {
        this.evaluateCollisionState();
        if (this.partController) {
          this.partController.updateCachedPartGrasp();
        }
        this.notifyState();
        this.requestRender();
      }
    );

    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();

    return result;
  }

  /**
   * 2c. Open to Clear
   */
  public openToClear(): { cleared: boolean; message: string } {
    const res = ContactGrip.openToClear(this.gripperModel, this.cadPartsRoot);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
    return res;
  }

  /**
   * 2c. Auto-place on outer wall
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

    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
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
    this.requestRender();
  }

  public flipApproach() {
    this.gripperModel.root.rotateX(Math.PI);
    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
  }

  public rotateAroundApproach(degrees: number) {
    const rad = THREE.MathUtils.degToRad(degrees);
    this.gripperModel.root.rotateZ(rad);
    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
  }

  public stepTranslate(axis: 'x' | 'y' | 'z', deltaMm: number) {
    this.gripperModel.root.position[axis] += deltaMm;
    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
  }

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
    this.requestRender();
  }

  /**
   * Sets numeric TCP pose in World coordinate frame (changes only T_world_gripper)
   */
  public setNumericPose(pos: { x: number; y: number; z: number }, rpy: EulerAngles) {
    this.gripperModel.root.position.set(pos.x, pos.y, pos.z);
    this.gripperModel.root.quaternion.copy(PoseMath.eulerRPYToQuaternion(rpy));
    this.gripperModel.root.updateMatrixWorld(true);

    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
  }

  /**
   * Places gripper at T_world_part * T_part_grasp (used when clicking saved grasp in list)
   */
  public setRelativeGraspPose(partGraspPose: Pose, rpy?: EulerAngles) {
    if (this.cadPartsRoot) {
      const world = PoseMath.computeWorldGraspTransform(this.cadPartsRoot.matrixWorld, partGraspPose);
      this.gripperModel.root.position.copy(world.position);
      this.gripperModel.root.quaternion.copy(world.quaternion);
    } else {
      this.gripperModel.root.position.set(
        partGraspPose.position.x,
        partGraspPose.position.y,
        partGraspPose.position.z
      );
      if (rpy) {
        this.gripperModel.root.quaternion.copy(PoseMath.eulerRPYToQuaternion(rpy));
      } else {
        this.gripperModel.root.quaternion.set(
          partGraspPose.orientation.x,
          partGraspPose.orientation.y,
          partGraspPose.orientation.z,
          partGraspPose.orientation.w
        );
      }
    }

    this.gripperModel.root.updateMatrixWorld(true);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
  }

  public setOpeningWidth(widthMm: number) {
    this.gripperModel.setOpeningWidth(widthMm);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
  }

  public setTcpOffset(offsetMm: number) {
    this.gripperModel.setTcpOffset(offsetMm);
    this.evaluateCollisionState();
    if (this.partController) {
      this.partController.updateCachedPartGrasp();
    }
    this.notifyState();
    this.requestRender();
  }

  public setStandoff(standoffMm: number) {
    this.standoff = Math.max(0, standoffMm);
    this.notifyState();
  }

  public setDepth(depthMm: number) {
    this.depth = Math.max(0, depthMm);
    this.notifyState();
  }

  public setTolerance(tolMm: number) {
    this.tolerance = Math.max(0.01, tolMm);
    this.evaluateCollisionState();
    this.notifyState();
    this.requestRender();
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
    this.requestRender();
  }

  /**
   * 2d. Test Grasp sequence with contact closing and lift
   */
  public testGrasp() {
    if (this.isTesting) return;

    this.isTesting = true;
    this.transformControls.enabled = false;
    this.transformControls.getHelper().visible = false;

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

    const approachVec = new THREE.Vector3(0, 0, 1).applyQuaternion(graspQuat).normalize();
    const preGraspPos = graspPos.clone().addScaledVector(approachVec, -this.preGraspDistance);

    const liftOffset = new THREE.Vector3(0, 0, 80);
    const liftPos = graspPos.clone().add(liftOffset);

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
        this.testPhase = 'pre-grasp';
        const t = Math.min(elapsed / phaseDurations.preGrasp, 1);
        const ease = 1 - Math.pow(1 - t, 2);
        this.gripperModel.root.position.lerpVectors(graspPos, preGraspPos, ease);
        this.gripperModel.setOpeningWidth(
          THREE.MathUtils.lerp(this.savedPoseBeforeTest!.openingWidth, GRIPPER_CONFIG.maxOpeningWidth, ease)
        );
      } else if (elapsed < tApproach) {
        this.testPhase = 'approach';
        const t = Math.min((elapsed - tPreGrasp) / phaseDurations.approach, 1);
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(preGraspPos, graspPos, ease);
        this.gripperModel.setOpeningWidth(GRIPPER_CONFIG.maxOpeningWidth);

        this.gripperModel.root.updateMatrixWorld(true);
        const colCheck = CollisionChecker.evaluateState(this.gripperModel, this.cadPartsRoot, this.tolerance);
        if (colCheck.hasCollision && t < 0.9) {
          this.contactStatus = 'collision';
          this.contactMessage = 'Collision during approach: open gripper hit CAD part!';
          this.gripperModel.highlightMeshes(colCheck.penetratingMeshes);
          this.notifyState();
          this.requestRender();
          setTimeout(() => this.stopTestGrasp(), 1800);
          return;
        }
      } else if (elapsed < tClose) {
        if (this.testPhase !== 'close') {
          this.testPhase = 'close';
          this.gripperModel.root.position.copy(graspPos);
          this.gripperModel.root.updateMatrixWorld(true);

          const closeRes = ContactGrip.computeContactGrasp(
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

          this.gripperModel.setFingerAngles(closeRes.targetLeftAngle, closeRes.targetRightAngle);
          isGraspAttached = closeRes.status === 'contact';
        }
      } else if (elapsed < tLift) {
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
        this.testPhase = 'release';
        const t = Math.min((elapsed - tPlace) / phaseDurations.release, 1);
        const ease = 1 - Math.pow(1 - t, 2);
        this.gripperModel.root.position.copy(placePos);
        this.gripperModel.setOpeningWidth(
          THREE.MathUtils.lerp(clampedTargetWidth, GRIPPER_CONFIG.maxOpeningWidth, ease)
        );
      } else if (elapsed < totalDuration) {
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
      this.requestRender();
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

    if (this.savedPoseBeforeTest) {
      this.gripperModel.root.position.copy(this.savedPoseBeforeTest.position);
      this.gripperModel.root.quaternion.copy(this.savedPoseBeforeTest.quaternion);
      this.gripperModel.setOpeningWidth(this.savedPoseBeforeTest.openingWidth);
      this.gripperModel.root.updateMatrixWorld(true);
    }

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
    this.requestRender();
  }

  /**
   * Computes current grasp state.
   * Shows: Part pose (world), Gripper pose (world), and read-only "Gripper relative to part" readout!
   */
  public getCurrentState(): GripperState {
    const worldPos = this.gripperModel.root.position;
    const worldQuat = this.gripperModel.root.quaternion;

    const worldPose: Pose = {
      position: {
        x: Number(worldPos.x.toFixed(3)),
        y: Number(worldPos.y.toFixed(3)),
        z: Number(worldPos.z.toFixed(3))
      },
      orientation: {
        x: Number(worldQuat.x.toFixed(4)),
        y: Number(worldQuat.y.toFixed(4)),
        z: Number(worldQuat.z.toFixed(4)),
        w: Number(worldQuat.w.toFixed(4))
      }
    };
    const worldRpy = PoseMath.quaternionToEulerRPY(worldQuat);

    let relativePose: Pose;
    let relativeRpy: EulerAngles;

    if (this.cadPartsRoot) {
      relativePose = PoseMath.computePartGraspPose(
        this.cadPartsRoot.matrixWorld,
        this.gripperModel.root.matrixWorld
      );
      this._scratchQuat.set(
        relativePose.orientation.x,
        relativePose.orientation.y,
        relativePose.orientation.z,
        relativePose.orientation.w
      );
      relativeRpy = PoseMath.quaternionToEulerRPY(this._scratchQuat);
    } else {
      relativePose = { ...worldPose };
      relativeRpy = { ...worldRpy };
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
      pose: worldPose,       // T_world_gripper
      rpy: worldRpy,
      relativePose,          // T_part_grasp
      relativeRpy,
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
