import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GripperModel } from './GripperModel';
import { CollisionChecker } from './collision';
import { GRIPPER_CONFIG } from '../config';
import type { Pose, EulerAngles, GripperState, TestPhase } from '../types';

export type GripperStateCallback = (state: GripperState) => void;

export class GraspController {
  private gripperModel: GripperModel;
  private transformControls: TransformControls;
  private orbitControls: OrbitControls;
  private scene: THREE.Scene;
  private cadPartsRoot: THREE.Group | null = null;

  private preGraspDistance = GRIPPER_CONFIG.defaultPreGraspDistance;
  private onStateChange: GripperStateCallback | null = null;
  private lastDetectedContactWidth = 0;

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
    orbitControls: OrbitControls
  ) {
    this.gripperModel = gripperModel;
    this.scene = scene;
    this.orbitControls = orbitControls;

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
      this.handleGizmoMove();
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

    // Initial default pose
    this.resetGripperPose();
  }

  public setCadPartsRoot(root: THREE.Group | null) {
    this.cadPartsRoot = root;
    this.updateContactWidth();
    this.checkCollisions();
  }

  public setOnStateChange(callback: GripperStateCallback) {
    this.onStateChange = callback;
    this.notifyState();
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
    // If gripper palm penetrates into CAD part, resolve standoff so it doesn't clip inside
    CollisionChecker.resolveApproachStandoff(this.gripperModel, this.cadPartsRoot, 10);
    this.updateContactWidth();
    this.checkCollisions();
    this.notifyState();
  }

  private updateContactWidth() {
    this.lastDetectedContactWidth = CollisionChecker.findGraspContactWidth(
      this.gripperModel,
      this.cadPartsRoot
    );
  }

  /**
   * Snaps TCP to clicked surface point with approach Z pointing into surface (Z = -normal)
   * Prevents gripper from penetrating inside the CAD model.
   */
  public snapToClickedPoint(
    point: { x: number; y: number; z: number },
    normal: { x: number; y: number; z: number }
  ) {
    // 1. Position TCP at the clicked surface point
    this.gripperModel.root.position.set(point.x, point.y, point.z);

    // 2. Approach direction: pointing INTO surface (Z = -normal)
    const targetZ = new THREE.Vector3(-normal.x, -normal.y, -normal.z).normalize();
    const defaultZ = new THREE.Vector3(0, 0, 1);

    const quat = new THREE.Quaternion();
    if (targetZ.distanceTo(new THREE.Vector3(0, 0, -1)) < 0.0001) {
      quat.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
    } else {
      quat.setFromUnitVectors(defaultZ, targetZ);
    }

    this.gripperModel.root.quaternion.copy(quat);
    this.gripperModel.root.updateMatrixWorld(true);

    // 3. Contact blocking: ensure the gripper body/palm does NOT penetrate inside the CAD model
    CollisionChecker.resolveApproachStandoff(this.gripperModel, this.cadPartsRoot, 80);

    this.updateContactWidth();
    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Flips approach direction 180 degrees
   */
  public flipApproach() {
    this.gripperModel.root.rotateX(Math.PI);
    this.gripperModel.root.updateMatrixWorld(true);
    CollisionChecker.resolveApproachStandoff(this.gripperModel, this.cadPartsRoot, 80);
    this.updateContactWidth();
    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Rotates around approach axis (+Z) by degrees
   */
  public rotateAroundApproach(degrees: number) {
    const rad = THREE.MathUtils.degToRad(degrees);
    this.gripperModel.root.rotateZ(rad);
    this.gripperModel.root.updateMatrixWorld(true);
    CollisionChecker.resolveApproachStandoff(this.gripperModel, this.cadPartsRoot, 40);
    this.updateContactWidth();
    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Translates along part axis by delta in mm with contact boundary resolution
   */
  public stepTranslate(axis: 'x' | 'y' | 'z', deltaMm: number) {
    this.gripperModel.root.position[axis] += deltaMm;
    this.gripperModel.root.updateMatrixWorld(true);

    // If body penetrates CAD part, push back to contact surface
    CollisionChecker.resolveApproachStandoff(this.gripperModel, this.cadPartsRoot, Math.abs(deltaMm) + 10);

    this.updateContactWidth();
    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Rotates around local axis by delta in degrees
   */
  public stepRotate(axis: 'x' | 'y' | 'z', deltaDeg: number) {
    const rad = THREE.MathUtils.degToRad(deltaDeg);
    if (axis === 'x') this.gripperModel.root.rotateX(rad);
    if (axis === 'y') this.gripperModel.root.rotateY(rad);
    if (axis === 'z') this.gripperModel.root.rotateZ(rad);
    this.gripperModel.root.updateMatrixWorld(true);

    CollisionChecker.resolveApproachStandoff(this.gripperModel, this.cadPartsRoot, 30);
    this.updateContactWidth();
    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Sets exact numeric TCP pose in part coordinate frame
   */
  public setNumericPose(pos: { x: number; y: number; z: number }, rpy: EulerAngles) {
    this.gripperModel.root.position.set(pos.x, pos.y, pos.z);

    const euler = new THREE.Euler(
      THREE.MathUtils.degToRad(rpy.roll),
      THREE.MathUtils.degToRad(rpy.pitch),
      THREE.MathUtils.degToRad(rpy.yaw),
      'XYZ'
    );
    this.gripperModel.root.quaternion.setFromEuler(euler);
    this.gripperModel.root.updateMatrixWorld(true);

    CollisionChecker.resolveApproachStandoff(this.gripperModel, this.cadPartsRoot, 60);
    this.updateContactWidth();
    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Closes or opens fingers. If CAD part is between fingers, stops at contact width.
   */
  public setOpeningWidth(widthMm: number) {
    this.updateContactWidth();
    let clampedWidth = widthMm;
    if (this.lastDetectedContactWidth > 0 && widthMm < this.lastDetectedContactWidth) {
      clampedWidth = this.lastDetectedContactWidth;
    }

    this.gripperModel.setOpeningWidth(clampedWidth);
    this.checkCollisions();
    this.notifyState();
  }

  public setTcpOffset(offsetMm: number) {
    this.gripperModel.setTcpOffset(offsetMm);
    CollisionChecker.resolveApproachStandoff(this.gripperModel, this.cadPartsRoot, 60);
    this.updateContactWidth();
    this.checkCollisions();
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
      // Position cleanly above part looking down along -Z
      this.gripperModel.root.position.set(center.x, center.y, box.max.z + 50);
      this.gripperModel.root.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
    } else {
      this.gripperModel.root.position.set(0, 0, 100);
      this.gripperModel.root.quaternion.set(0, 0, 0, 1);
    }

    this.gripperModel.setOpeningWidth(GRIPPER_CONFIG.defaultOpeningWidth);
    this.gripperModel.root.updateMatrixWorld(true);
    this.updateContactWidth();
    this.checkCollisions();
    this.notifyState();
  }

  public checkCollisions(): boolean {
    const res = CollisionChecker.checkCollision(
      this.gripperModel.getAllMeshes(),
      this.cadPartsRoot
    );
    this.gripperModel.setCollisionHighlight(res.hasCollision);
    return res.hasCollision;
  }

  /**
   * Runs the full animated Pick & Place sequence:
   * 1. Pre-grasp: Standoff at preGraspDistance with fingers open
   * 2. Approach: Linear approach along +Z (blocked at CAD surface)
   * 3. Close: Clamp on CAD part surface (stopping at contact width)
   * 4. Lift: PICK UP THE CAD MODEL into the air!
   * 5. Transfer: Carry CAD model to place location
   * 6. Place: Lower CAD model onto table/ground
   * 7. Release: Open fingers
   * 8. Retract: Retract gripper back to safe standoff
   * 9. Restore: Smoothly return CAD model and gripper to editable poses
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

    // 1. Ensure grasp pose is blocked on surface and does not penetrate inside CAD solid
    CollisionChecker.resolveApproachStandoff(this.gripperModel, this.cadPartsRoot, 80);
    const graspPos = this.gripperModel.root.position.clone();
    const graspQuat = this.gripperModel.root.quaternion.clone();

    // 2. Find exact workpiece contact width so fingers clamp on surface without penetrating
    const contactWidth = CollisionChecker.findGraspContactWidth(
      this.gripperModel,
      this.cadPartsRoot
    );
    const clampedTargetWidth =
      contactWidth > 0
        ? contactWidth
        : Math.min(this.savedPoseBeforeTest.openingWidth, 0);

    this.lastDetectedContactWidth = contactWidth;

    // Approach vector in world coordinates (+Z of TCP)
    const approachVec = new THREE.Vector3(0, 0, 1).applyQuaternion(graspQuat).normalize();
    const preGraspPos = graspPos.clone().addScaledVector(approachVec, -this.preGraspDistance);

    // Pick & Lift target: lifts 80mm vertically along world Z
    const liftOffset = new THREE.Vector3(0, 0, 80);
    const liftPos = graspPos.clone().add(liftOffset);

    // Place location (transferred +70mm in X)
    const transferOffset = new THREE.Vector3(70, 0, 80);
    const transferPos = graspPos.clone().add(transferOffset);
    const placePos = graspPos.clone().add(new THREE.Vector3(70, 0, 0));

    const startTime = performance.now();
    const phaseDurations = {
      preGrasp: 700,   // ms: retract & open
      approach: 900,   // ms: approach to contact boundary
      close: 600,      // ms: clamp on CAD part surface
      lift: 900,       // ms: PICK UP CAD MODEL into the air!
      transfer: 800,   // ms: carry CAD model horizontally
      place: 700,      // ms: place down on table
      release: 500,    // ms: release fingers
      retract: 700     // ms: lift gripper alone back to clearance
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
        // Phase 1: Pre-Grasp (open fingers and move to standoff)
        this.testPhase = 'pre-grasp';
        const t = Math.min(elapsed / phaseDurations.preGrasp, 1);
        const ease = 1 - Math.pow(1 - t, 2);
        this.gripperModel.root.position.lerpVectors(graspPos, preGraspPos, ease);
        this.gripperModel.setOpeningWidth(
          THREE.MathUtils.lerp(clampedTargetWidth, GRIPPER_CONFIG.maxOpeningWidth, ease)
        );
      } else if (elapsed < tApproach) {
        // Phase 2: Approach (move linearly into grasp pose, stopping at contact boundary)
        this.testPhase = 'approach';
        const t = Math.min((elapsed - tPreGrasp) / phaseDurations.approach, 1);
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(preGraspPos, graspPos, ease);
        this.gripperModel.setOpeningWidth(GRIPPER_CONFIG.maxOpeningWidth);
      } else if (elapsed < tClose) {
        // Phase 3: Close (clamp fingers on CAD part surface, stopping at contact width)
        this.testPhase = 'close';
        const t = Math.min((elapsed - tApproach) / phaseDurations.close, 1);
        const ease = 1 - Math.pow(1 - t, 2);
        this.gripperModel.root.position.copy(graspPos);
        this.gripperModel.setOpeningWidth(
          THREE.MathUtils.lerp(GRIPPER_CONFIG.maxOpeningWidth, clampedTargetWidth, ease)
        );
      } else if (elapsed < tLift) {
        // Phase 4: Lift (PICK UP THE CAD MODEL!)
        this.testPhase = 'lift';
        const t = Math.min((elapsed - tClose) / phaseDurations.lift, 1);
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(graspPos, liftPos, ease);
        this.gripperModel.setOpeningWidth(clampedTargetWidth);

        // CAD MODEL LIFTS WITH THE GRIPPER!
        if (this.cadPartsRoot && this.savedCadPosBeforeTest) {
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

        if (this.cadPartsRoot && this.savedCadPosBeforeTest) {
          const delta = this.gripperModel.root.position.clone().sub(graspPos);
          this.cadPartsRoot.position.copy(this.savedCadPosBeforeTest).add(delta);
          this.cadPartsRoot.updateMatrixWorld(true);
        }
      } else if (elapsed < tPlace) {
        // Phase 6: Place (lower CAD model down to resting surface)
        this.testPhase = 'place';
        const t = Math.min((elapsed - tTransfer) / phaseDurations.place, 1);
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(transferPos, placePos, ease);
        this.gripperModel.setOpeningWidth(clampedTargetWidth);

        if (this.cadPartsRoot && this.savedCadPosBeforeTest) {
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
        // Sequence completed: brief pause to display placed result, then restore
        setTimeout(() => {
          this.stopTestGrasp();
        }, 1200);
        return;
      }

      this.gripperModel.root.updateMatrixWorld(true);
      this.checkCollisions();
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

    this.updateContactWidth();
    this.checkCollisions();
    this.notifyState();
  }

  public getCurrentState(): GripperState {
    const pos = this.gripperModel.root.position;
    const quat = this.gripperModel.root.quaternion;

    const euler = new THREE.Euler().setFromQuaternion(quat, 'XYZ');
    const rpy: EulerAngles = {
      roll: Number(THREE.MathUtils.radToDeg(euler.x).toFixed(2)),
      pitch: Number(THREE.MathUtils.radToDeg(euler.y).toFixed(2)),
      yaw: Number(THREE.MathUtils.radToDeg(euler.z).toFixed(2))
    };

    const tcpPose: Pose = {
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

    return {
      loaded: true,
      visible: this.gripperModel.isVisible(),
      openingWidth: Number(this.gripperModel.getOpeningWidth().toFixed(1)),
      contactWidth:
        this.lastDetectedContactWidth > 0
          ? Number(this.lastDetectedContactWidth.toFixed(1))
          : undefined,
      tcpOffset: Number(this.gripperModel.getTcpOffset().toFixed(1)),
      preGraspDistance: Number(this.preGraspDistance.toFixed(1)),
      pose: tcpPose,
      rpy,
      hasCollision: this.gripperModel.getIsColliding(),
      isTesting: this.isTesting,
      testPhase: this.testPhase
    };
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
