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

  // Animation test sequence state
  private isTesting = false;
  private testPhase: TestPhase = 'idle';
  private animationId: number | null = null;
  private savedPoseBeforeTest: {
    position: THREE.Vector3;
    quaternion: THREE.Quaternion;
    openingWidth: number;
  } | null = null;

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
      // Do not trigger if typing in an input or textarea
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
    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Snaps TCP to clicked surface point with approach Z pointing into surface (Z = -normal)
   */
  public snapToClickedPoint(point: { x: number; y: number; z: number }, normal: { x: number; y: number; z: number }) {
    // 1. Position TCP directly on the clicked surface point
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

    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Flips approach direction 180 degrees
   */
  public flipApproach() {
    this.gripperModel.root.rotateX(Math.PI);
    this.gripperModel.root.updateMatrixWorld(true);
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
    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Translates along part axis by delta in mm
   */
  public stepTranslate(axis: 'x' | 'y' | 'z', deltaMm: number) {
    this.gripperModel.root.position[axis] += deltaMm;
    this.gripperModel.root.updateMatrixWorld(true);
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

    this.checkCollisions();
    this.notifyState();
  }

  public setOpeningWidth(widthMm: number) {
    this.gripperModel.setOpeningWidth(widthMm);
    this.checkCollisions();
    this.notifyState();
  }

  public setTcpOffset(offsetMm: number) {
    this.gripperModel.setTcpOffset(offsetMm);
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
      const size = box.getSize(new THREE.Vector3());
      // Position above part looking down along -Z
      this.gripperModel.root.position.set(center.x, center.y, box.max.z + 50);
      this.gripperModel.root.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
    } else {
      this.gripperModel.root.position.set(0, 0, 100);
      this.gripperModel.root.quaternion.set(0, 0, 0, 1);
    }

    this.gripperModel.setOpeningWidth(GRIPPER_CONFIG.defaultOpeningWidth);
    this.gripperModel.root.updateMatrixWorld(true);
    this.checkCollisions();
    this.notifyState();
  }

  /**
   * Check for collisions with CAD part meshes
   */
  public checkCollisions(): boolean {
    const res = CollisionChecker.checkCollision(
      this.gripperModel.getAllMeshes(),
      this.cadPartsRoot
    );
    this.gripperModel.setCollisionHighlight(res.hasCollision);
    return res.hasCollision;
  }

  /**
   * Runs the animated test grasp sequence:
   * 1. Pre-grasp (retract along -Z, open fingers)
   * 2. Approach (move along +Z to grasp pose)
   * 3. Close (close fingers)
   * 4. Lift (retract back to pre-grasp)
   * 5. Restore editable pose
   */
  public testGrasp() {
    if (this.isTesting) return;

    this.isTesting = true;
    this.transformControls.enabled = false;
    this.transformControls.getHelper().visible = false;

    // Save initial editable grasp pose
    this.savedPoseBeforeTest = {
      position: this.gripperModel.root.position.clone(),
      quaternion: this.gripperModel.root.quaternion.clone(),
      openingWidth: this.gripperModel.getOpeningWidth()
    };

    const graspPos = this.savedPoseBeforeTest.position.clone();
    const graspQuat = this.savedPoseBeforeTest.quaternion.clone();
    const targetWidth = this.savedPoseBeforeTest.openingWidth;

    // Approach vector in world coordinates (+Z of TCP)
    const approachVec = new THREE.Vector3(0, 0, 1).applyQuaternion(graspQuat).normalize();
    const preGraspPos = graspPos.clone().addScaledVector(approachVec, -this.preGraspDistance);

    const startTime = performance.now();
    const phaseDurations = {
      preGrasp: 800,  // ms
      approach: 1000, // ms
      close: 600,     // ms
      lift: 900       // ms
    };

    const totalDuration =
      phaseDurations.preGrasp +
      phaseDurations.approach +
      phaseDurations.close +
      phaseDurations.lift;

    const animateStep = (now: number) => {
      if (!this.isTesting) return;

      const elapsed = now - startTime;

      if (elapsed < phaseDurations.preGrasp) {
        // Phase 1: Pre-Grasp (open fingers and move to pre-grasp position)
        this.testPhase = 'pre-grasp';
        const t = Math.min(elapsed / phaseDurations.preGrasp, 1);
        const ease = 1 - Math.pow(1 - t, 2);
        this.gripperModel.root.position.lerpVectors(graspPos, preGraspPos, ease);
        this.gripperModel.setOpeningWidth(
          THREE.MathUtils.lerp(targetWidth, GRIPPER_CONFIG.maxOpeningWidth, ease)
        );
      } else if (elapsed < phaseDurations.preGrasp + phaseDurations.approach) {
        // Phase 2: Approach (move linearly into grasp position)
        this.testPhase = 'approach';
        const t = Math.min(
          (elapsed - phaseDurations.preGrasp) / phaseDurations.approach,
          1
        );
        const ease = t * t * (3 - 2 * t); // smoothstep
        this.gripperModel.root.position.lerpVectors(preGraspPos, graspPos, ease);
        this.gripperModel.setOpeningWidth(GRIPPER_CONFIG.maxOpeningWidth);
      } else if (
        elapsed <
        phaseDurations.preGrasp + phaseDurations.approach + phaseDurations.close
      ) {
        // Phase 3: Close fingers onto part
        this.testPhase = 'close';
        const t = Math.min(
          (elapsed - phaseDurations.preGrasp - phaseDurations.approach) /
            phaseDurations.close,
          1
        );
        const ease = 1 - Math.pow(1 - t, 2);
        this.gripperModel.root.position.copy(graspPos);
        this.gripperModel.setOpeningWidth(
          THREE.MathUtils.lerp(GRIPPER_CONFIG.maxOpeningWidth, targetWidth, ease)
        );
      } else if (elapsed < totalDuration) {
        // Phase 4: Lift back to pre-grasp
        this.testPhase = 'lift';
        const t = Math.min(
          (elapsed -
            phaseDurations.preGrasp -
            phaseDurations.approach -
            phaseDurations.close) /
            phaseDurations.lift,
          1
        );
        const ease = t * t * (3 - 2 * t);
        this.gripperModel.root.position.lerpVectors(graspPos, preGraspPos, ease);
        this.gripperModel.setOpeningWidth(targetWidth);
      } else {
        // Completed: wait briefly and restore editable pose
        setTimeout(() => {
          this.stopTestGrasp();
        }, 400);
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

    this.transformControls.enabled = true;
    this.transformControls.getHelper().visible = true;

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
