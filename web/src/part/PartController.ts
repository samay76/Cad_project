import * as THREE from 'three';
import { PoseMath } from '../math/poses';
import { CollisionChecker } from '../gripper/collision';
import type { EulerAngles, PartPoseState } from '../types';

export type PartPoseCallback = (state: PartPoseState) => void;

export class PartController {
  private cadPartsRoot: THREE.Group | null = null;
  private gripperRoot: THREE.Group | null = null;
  private onStateChange: PartPoseCallback | null = null;
  private onGripperRelativeChanged: (() => void) | null = null;

  // "Move gripper together with part": default OFF (false)
  private moveGripperTogetherWithPart = false;

  // Cached T_part_grasp relative transform matrix (inv(T_world_part) * T_world_gripper)
  private cachedT_part_grasp = new THREE.Matrix4();
  private initialPartMatrix = new THREE.Matrix4();

  // Reusable objects to avoid per-frame allocations
  private _pos = new THREE.Vector3();
  private _quat = new THREE.Quaternion();
  private _scale = new THREE.Vector3();
  private _mat = new THREE.Matrix4();

  constructor() {}

  public setCadPartsRoot(root: THREE.Group | null) {
    this.cadPartsRoot = root;
    this.assertSiblings();
    if (this.cadPartsRoot) {
      this.initialPartMatrix.copy(this.cadPartsRoot.matrix);
      this.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  public setGripperRoot(root: THREE.Group | null) {
    this.gripperRoot = root;
    this.assertSiblings();
    if (this.cadPartsRoot && this.gripperRoot) {
      this.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  /**
   * Enforces that partRoot and gripperRoot are strictly siblings under the scene
   */
  private assertSiblings() {
    if (this.cadPartsRoot && this.gripperRoot) {
      if (
        this.cadPartsRoot.parent === this.gripperRoot ||
        this.gripperRoot.parent === this.cadPartsRoot
      ) {
        console.error('Scene parenting error: partRoot and gripperRoot must be siblings under the scene root!');
      }
    }
  }

  public setOnStateChange(callback: PartPoseCallback) {
    this.onStateChange = callback;
    this.notifyState();
  }

  public setOnGripperRelativeChanged(callback: () => void) {
    this.onGripperRelativeChanged = callback;
  }

  /**
   * Checkbox "Move gripper together with part" (default OFF)
   */
  public setMoveGripperTogetherWithPart(moveTogether: boolean) {
    this.moveGripperTogetherWithPart = moveTogether;
    if (moveTogether) {
      this.updateCachedPartGrasp();
    }
    this.notifyState();
  }

  // Alias for backward compatibility
  public setLockGripperToPart(locked: boolean) {
    this.setMoveGripperTogetherWithPart(locked);
  }

  public getMoveGripperTogetherWithPart(): boolean {
    return this.moveGripperTogetherWithPart;
  }

  public getLockGripperToPart(): boolean {
    return this.moveGripperTogetherWithPart;
  }

  /**
   * Caches T_part_grasp = inv(T_world_part) * T_world_gripper
   */
  public updateCachedPartGrasp() {
    if (!this.cadPartsRoot || !this.gripperRoot) return;
    this.cadPartsRoot.updateMatrixWorld(true);
    this.gripperRoot.updateMatrixWorld(true);

    PoseMath.computeT_part_grasp(
      this.cadPartsRoot.matrixWorld,
      this.gripperRoot.matrixWorld,
      this.cachedT_part_grasp
    );
  }

  /**
   * Called whenever CAD part pose changes (via gizmo, numeric controls, or step buttons).
   * ONLY when "Move gripper together with part" is ON may the gripper follow the part.
   * When OFF (default), ONLY the CAD model moves; the gripper stays untouched in world coordinates.
   */
  public handlePartMoved() {
    if (!this.cadPartsRoot) return;
    this.cadPartsRoot.updateMatrixWorld(true);

    if (this.moveGripperTogetherWithPart && this.gripperRoot) {
      // T_world_gripper = T_world_part * T_part_grasp
      PoseMath.computeT_world_gripper(
        this.cadPartsRoot.matrixWorld,
        this.cachedT_part_grasp,
        this._mat
      );

      this._mat.decompose(this._pos, this._quat, this._scale);

      this.gripperRoot.position.copy(this._pos);
      this.gripperRoot.quaternion.copy(this._quat);
      this.gripperRoot.updateMatrixWorld(true);
    } else {
      // Move gripper together is OFF: part moves alone, gripper stays fixed in world!
      // Update the relative grasp matrix since part moved relative to fixed gripper
      this.updateCachedPartGrasp();
    }

    if (this.onGripperRelativeChanged) {
      this.onGripperRelativeChanged();
    }

    this.notifyState();
  }

  /**
   * Translates CAD part along an axis (changes only T_world_part)
   */
  public stepTranslate(axis: 'x' | 'y' | 'z', deltaMm: number) {
    if (!this.cadPartsRoot) return;
    this.cadPartsRoot.position[axis] += deltaMm;
    this.handlePartMoved();
  }

  /**
   * Rotates CAD part around world or local axis (changes only T_world_part)
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
   * Sets exact numeric pose for CAD part in world frame (changes only T_world_part)
   */
  public setNumericPose(pos: { x: number; y: number; z: number }, rpy: EulerAngles) {
    if (!this.cadPartsRoot) return;
    this.cadPartsRoot.position.set(pos.x, pos.y, pos.z);

    const quat = PoseMath.eulerRPYToQuaternion(rpy, this._quat);
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

    // 4. Invalidate collision cache on CAD part
    CollisionChecker.invalidateCadMeshCache();
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
        moveGripperTogetherWithPart: this.moveGripperTogetherWithPart,
        lockGripperToPart: this.moveGripperTogetherWithPart
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
      moveGripperTogetherWithPart: this.moveGripperTogetherWithPart,
      lockGripperToPart: this.moveGripperTogetherWithPart
    };
  }

  private notifyState() {
    if (this.onStateChange) {
      this.onStateChange(this.getCurrentState());
    }
  }
}
