import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GripperModel } from './GripperModel';
import { CollisionChecker, type DetailedCollisionResult } from './collision';
import { PoseMath } from '../math/poses';
import { PartController } from '../part/PartController';
import { GRIPPER_CONFIG } from '../config';
import type {
  Pose,
  EulerAngles,
  GripperState,
  TestPhase,
  ContactStatusType
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

  public setPartController(controller: PartController) {
    this.partController = controller;
    this.partController.setGripperRoot(this.gripperModel.root);
    if (this.cadPartsRoot) {
      this.partController.setCadPartsRoot(this.cadPartsRoot);
    }
  }

  public setCadPartsRoot(root: THREE.Group | null) {
    const firstCadLoad = !this.cadPartsRoot && !!root;
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

    if (firstCadLoad) {
      this.resetGripperPose();
      return;
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

  /** Shows or hides the gripper and its gizmo (hidden while the live cell is shown). */
  public setVisible(visible: boolean) {
    this.gripperModel.root.visible = visible;
    this.setTransformVisible(visible);
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
      this.gripperModel.root.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
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

  /** Hands the gripper and part to a playback (AutoPickPreview). stopTestGrasp() restores both. */
  public beginExternalMotion() {
    if (this.isTesting) this.stopTestGrasp();
    this.isTesting = true;
    this.transformControls.enabled = false;
    this.transformControls.getHelper().visible = false;
    this.gripperModel.highlightMeshes([]);
    this.savedPoseBeforeTest = {
      position: this.gripperModel.root.position.clone(),
      quaternion: this.gripperModel.root.quaternion.clone(),
      openingWidth: this.gripperModel.getOpeningWidth()
    };
    if (this.cadPartsRoot) {
      this.savedCadPosBeforeTest = this.cadPartsRoot.position.clone();
      this.savedCadQuatBeforeTest = this.cadPartsRoot.quaternion.clone();
    }
  }

  public applyExternalMotion(position: THREE.Vector3, quaternion: THREE.Quaternion, widthMm: number) {
    this.gripperModel.root.position.copy(position);
    this.gripperModel.root.quaternion.copy(quaternion);
    this.gripperModel.setOpeningWidth(widthMm);
    this.gripperModel.root.updateMatrixWorld(true);
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
