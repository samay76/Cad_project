import * as THREE from 'three';
import type { Pose, EulerAngles, Vector3D } from '../types';

/**
 * Centralized Pose & Coordinate Frame Transform Math
 * 
 * Required Transforms:
 * - T_world_part: Transform/Matrix of CAD Part frame in World space
 * - T_world_gripper: Transform/Matrix of Gripper TCP frame in World space
 * - T_part_grasp: Relative transform of Gripper TCP in CAD Part local frame
 * 
 * Relations:
 *   T_part_grasp = inverse(T_world_part) * T_world_gripper
 *   T_world_gripper = T_world_part * T_part_grasp
 */

export class PoseMath {
  // Reusable scratch objects for allocation-free hot path computations
  private static _pos = new THREE.Vector3();
  private static _quat = new THREE.Quaternion();
  private static _scale = new THREE.Vector3();
  private static _euler = new THREE.Euler(0, 0, 0, 'XYZ');
  private static _invMat = new THREE.Matrix4();
  private static _scratchMat = new THREE.Matrix4();
  private static _scratchMat2 = new THREE.Matrix4();

  /**
   * Gets T_world_part matrix from cadPartsRoot
   */
  public static getT_world_part(cadPartsRoot: THREE.Object3D | null): THREE.Matrix4 {
    if (!cadPartsRoot) return this._scratchMat.identity();
    cadPartsRoot.updateMatrixWorld(true);
    return cadPartsRoot.matrixWorld;
  }

  /**
   * Gets T_world_gripper matrix from gripperRoot (TCP)
   */
  public static getT_world_gripper(gripperRoot: THREE.Object3D | null): THREE.Matrix4 {
    if (!gripperRoot) return this._scratchMat.identity();
    gripperRoot.updateMatrixWorld(true);
    return gripperRoot.matrixWorld;
  }

  /**
   * Computes on demand: T_part_grasp = inverse(T_world_part) * T_world_gripper
   */
  public static computeT_part_grasp(
    T_world_part: THREE.Matrix4,
    T_world_gripper: THREE.Matrix4,
    out = new THREE.Matrix4()
  ): THREE.Matrix4 {
    this._invMat.copy(T_world_part).invert();
    return out.multiplyMatrices(this._invMat, T_world_gripper);
  }

  /**
   * Computes: T_world_gripper = T_world_part * T_part_grasp
   */
  public static computeT_world_gripper(
    T_world_part: THREE.Matrix4,
    T_part_grasp: THREE.Matrix4,
    out = new THREE.Matrix4()
  ): THREE.Matrix4 {
    return out.multiplyMatrices(T_world_part, T_part_grasp);
  }

  /**
   * Converts a Three.js Matrix4 to a serializable Pose (position xyz in mm + quaternion xyzw)
   */
  public static matrixToPose(matrix: THREE.Matrix4): Pose {
    matrix.decompose(this._pos, this._quat, this._scale);

    return {
      position: {
        x: Number(this._pos.x.toFixed(3)),
        y: Number(this._pos.y.toFixed(3)),
        z: Number(this._pos.z.toFixed(3))
      },
      orientation: {
        x: Number(this._quat.x.toFixed(4)),
        y: Number(this._quat.y.toFixed(4)),
        z: Number(this._quat.z.toFixed(4)),
        w: Number(this._quat.w.toFixed(4))
      }
    };
  }

  /**
   * Converts a Pose to a Three.js Matrix4
   */
  public static poseToMatrix(pose: Pose, out = new THREE.Matrix4()): THREE.Matrix4 {
    this._pos.set(pose.position.x, pose.position.y, pose.position.z);
    this._quat.set(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
    );
    this._scale.set(1, 1, 1);
    return out.compose(this._pos, this._quat, this._scale);
  }

  /**
   * Converts a quaternion to Euler Roll, Pitch, Yaw in degrees (XYZ order)
   */
  public static quaternionToEulerRPY(quat: THREE.Quaternion): EulerAngles {
    this._euler.setFromQuaternion(quat, 'XYZ');
    return {
      roll: Number(THREE.MathUtils.radToDeg(this._euler.x).toFixed(2)),
      pitch: Number(THREE.MathUtils.radToDeg(this._euler.y).toFixed(2)),
      yaw: Number(THREE.MathUtils.radToDeg(this._euler.z).toFixed(2))
    };
  }

  /**
   * Converts Euler Roll, Pitch, Yaw in degrees to a Three.js Quaternion
   */
  public static eulerRPYToQuaternion(rpy: EulerAngles, out = new THREE.Quaternion()): THREE.Quaternion {
    this._euler.set(
      THREE.MathUtils.degToRad(rpy.roll),
      THREE.MathUtils.degToRad(rpy.pitch),
      THREE.MathUtils.degToRad(rpy.yaw),
      'XYZ'
    );
    return out.setFromEuler(this._euler);
  }

  /**
   * Computes T_part_grasp relative matrix: alias for computeT_part_grasp
   */
  public static computePartGraspMatrix(
    T_world_part: THREE.Matrix4,
    T_world_grasp: THREE.Matrix4
  ): THREE.Matrix4 {
    return this.computeT_part_grasp(T_world_part, T_world_grasp);
  }

  /**
   * Computes T_part_grasp as a Pose
   */
  public static computePartGraspPose(
    T_world_part: THREE.Matrix4,
    T_world_grasp: THREE.Matrix4
  ): Pose {
    this.computeT_part_grasp(T_world_part, T_world_grasp, this._scratchMat);
    return this.matrixToPose(this._scratchMat);
  }

  /**
   * Computes T_world_grasp = T_world_part * T_part_grasp
   */
  public static computeWorldGraspMatrix(
    T_world_part: THREE.Matrix4,
    T_part_grasp: THREE.Matrix4
  ): THREE.Matrix4 {
    return this.computeT_world_gripper(T_world_part, T_part_grasp);
  }

  /**
   * Computes world position and quaternion from T_world_part and T_part_grasp (Pose)
   */
  public static computeWorldGraspTransform(
    T_world_part: THREE.Matrix4,
    graspPose: Pose
  ): { position: THREE.Vector3; quaternion: THREE.Quaternion } {
    this.poseToMatrix(graspPose, this._scratchMat2);
    this.computeT_world_gripper(T_world_part, this._scratchMat2, this._scratchMat);

    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    this._scratchMat.decompose(pos, quat, this._scale);

    return { position: pos, quaternion: quat };
  }

  /**
   * Transforms a 3D point from World space to CAD Part local frame
   */
  public static worldPointToPartFrame(
    worldPoint: THREE.Vector3,
    T_world_part: THREE.Matrix4
  ): Vector3D {
    this._invMat.copy(T_world_part).invert();
    this._pos.copy(worldPoint).applyMatrix4(this._invMat);
    return {
      x: Number(this._pos.x.toFixed(3)),
      y: Number(this._pos.y.toFixed(3)),
      z: Number(this._pos.z.toFixed(3))
    };
  }

  /**
   * Transforms a 3D point from CAD Part local frame to World space
   */
  public static partPointToWorldFrame(
    partPoint: Vector3D,
    T_world_part: THREE.Matrix4
  ): THREE.Vector3 {
    return new THREE.Vector3(partPoint.x, partPoint.y, partPoint.z).applyMatrix4(T_world_part);
  }
}
