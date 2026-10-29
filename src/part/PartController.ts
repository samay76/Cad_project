import * as THREE from 'three';
import { PoseMath } from '../math/poses';
import { CollisionChecker } from '../gripper/collision';
import type { Pose, EulerAngles, PartPoseState } from '../types';

export type PartPoseCallback = (state: PartPoseState) => void;

export class PartController {
  private cadPartsRoot: THREE.Group | null = null;
  private gripperRoot: THREE.Group | null = null;
  private onStateChange: PartPoseCallback | null = null;

  // "Lock gripper to part": default ON
  private lockGripperToPart = true;

  // Cached T_part_grasp relative transform matrix
  private cachedT_part_grasp = new THREE.Matrix4();
  private initialPartMatrix = new THREE.Matrix4();

  constructor() {}

  public setCadPartsRoot(root: THREE.Group | null) {
    this.cadPartsRoot = root;
    if (this.cadPartsRoot) {
      this.initialPartMatrix.copy(this.cadPartsRoot.matrix);
      this.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  public setGripperRoot(root: THREE.Group | null) {
    this.gripperRoot = root;
    if (this.cadPartsRoot && this.gripperRoot) {
      this.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  public setOnStateChange(callback: PartPoseCallback) {
    this.onStateChange = callback;
    this.notifyState();
  }

  public setLockGripperToPart(locked: boolean) {
    this.lockGripperToPart = locked;
    if (locked) {
      this.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  public getLockGripperToPart(): boolean {
    return this.lockGripperToPart;
  }

  /**
   * Caches T_part_grasp = inv(T_world_part) * T_world_grasp
   */
  public updateCachedPartGrasp() {
    if (!this.cadPartsRoot || !this.gripperRoot) return;
    this.cadPartsRoot.updateMatrixWorld(true);
    this.gripperRoot.updateMatrixWorld(true);

    this.cachedT_part_grasp = PoseMath.computePartGraspMatrix(
      this.cadPartsRoot.matrixWorld,
      this.gripperRoot.matrixWorld
    );
  }

  /**
   * Called whenever CAD part pose changes (via gizmo or numeric controls).
   * If lockGripperToPart is ON, moves the gripper synchronously so T_part_grasp does not change.
   */
  public handlePartMoved() {
    if (!this.cadPartsRoot) return;
    this.cadPartsRoot.updateMatrixWorld(true);

    if (this.lockGripperToPart && this.gripperRoot) {
      // T_world_grasp = T_world_part * T_part_grasp
      const T_world_grasp = PoseMath.computeWorldGraspMatrix(
        this.cadPartsRoot.matrixWorld,
        this.cachedT_part_grasp
      );

      const pos = new THREE.Vector3();
      const quat = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      T_world_grasp.decompose(pos, quat, scale);

      this.gripperRoot.position.copy(pos);
      this.gripperRoot.quaternion.copy(quat);
      this.gripperRoot.updateMatrixWorld(true);
    } else {
      // Lock is OFF: part moves alone, update cached grasp relative transform
      this.updateCachedPartGrasp();
    }

    this.notifyState();
  }

  /**
   * Translates CAD part along an axis
   */
  public stepTranslate(axis: 'x' | 'y' | 'z', deltaMm: number) {
    if (!this.cadPartsRoot) return;
    this.cadPartsRoot.position[axis] += deltaMm;
    this.handlePartMoved();
  }

  /**
   * Rotates CAD part around world or local axis
   */
  public stepRotate(axis: 'x' | 'y' | 'z', deltaDeg: number) {
    if (!this.cadPartsRoot) return;
    const rad = THREE.MathUtils.degToRad(deltaDeg);
    if (axis === 'x') this.cadPartsRoot.rotateX(rad);
    if (axis === 'y') this.cadPartsRoot.rotateY(rad);
    if (axis === 'z') this.cadPartsRoot.rotateZ(rad);
    this.handlePartMoved();
  }

  /**
   * Sets exact numeric pose for CAD part
   */
  public setNumericPose(pos: { x: number; y: number; z: number }, rpy: EulerAngles) {
    if (!this.cadPartsRoot) return;
    this.cadPartsRoot.position.set(pos.x, pos.y, pos.z);

    const quat = PoseMath.eulerRPYToQuaternion(rpy);
    this.cadPartsRoot.quaternion.copy(quat);
    this.handlePartMoved();
  }

  /**
   * Resets CAD part pose to initial orientation / origin
   */
  public resetPartPose() {
    if (!this.cadPartsRoot) return;
    this.cadPartsRoot.position.set(0, 0, 0);
    this.cadPartsRoot.quaternion.set(0, 0, 0, 1);
    this.cadPartsRoot.scale.set(1, 1, 1);
    this.handlePartMoved();
  }

  /**
   * Bakes the current part rotation into the model frame.
   * Warns that existing saved grasps will be invalid and asks for confirmation.
   */
  public bakeCurrentOrientationAsOrigin(
    onSuccess?: () => void,
    onCancel?: () => void
  ): boolean {
    if (!this.cadPartsRoot) return false;

    const confirmed = window.confirm(
      'Warning: Baking the current rotation into the model origin will modify the CAD coordinate frame. ' +
        'Existing saved grasp poses for this model will become invalid.\n\n' +
        'Do you want to proceed and set current orientation as origin?'
    );

    if (!confirmed) {
      if (onCancel) onCancel();
      return false;
    }

    // 1. Compute rotation transform matrix
    const rotMat = new THREE.Matrix4().makeRotationFromQuaternion(this.cadPartsRoot.quaternion);

    // 2. Apply rotation to all child mesh geometries
    this.cadPartsRoot.traverse((child) => {
      if (child instanceof THREE.Mesh && child.geometry) {
        child.geometry.applyMatrix4(rotMat);
        child.geometry.computeVertexNormals();
        child.geometry.computeBoundingBox();
        child.geometry.computeBoundingSphere();
      }
    });

    // 3. Reset part rotation back to identity
    this.cadPartsRoot.quaternion.set(0, 0, 0, 1);
    this.cadPartsRoot.updateMatrixWorld(true);

    // 4. Rebuild BVH bounds trees because geometry vertices moved
    CollisionChecker.ensureBVH(this.cadPartsRoot, true);

    // 5. Update cached part grasp
    this.updateCachedPartGrasp();
    this.notifyState();

    if (onSuccess) onSuccess();
    return true;
  }

  public getCurrentState(): PartPoseState {
    if (!this.cadPartsRoot) {
      return {
        pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 }
        },
        rpy: { roll: 0, pitch: 0, yaw: 0 },
        lockGripperToPart: this.lockGripperToPart
      };
    }

    const pos = this.cadPartsRoot.position;
    const quat = this.cadPartsRoot.quaternion;
    const rpy = PoseMath.quaternionToEulerRPY(quat);

    return {
      pose: {
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
      },
      rpy,
      lockGripperToPart: this.lockGripperToPart
    };
  }

  private notifyState() {
    if (this.onStateChange) {
      this.onStateChange(this.getCurrentState());
    }
  }
}
