import * as THREE from 'three';
import type { Pose, EulerAngles, Vector3D, QuaternionD } from '../types';

/**
 * Centralized Pose & Coordinate Frame Transform Math
 * Conventions:
 * - T_world_part: Pose/Matrix of CAD Part frame relative to World frame
 * - T_world_grasp: Pose/Matrix of Gripper TCP frame relative to World frame
 * - T_part_grasp: Pose/Matrix of Gripper TCP expressed relative to CAD Part frame
 * 
 * Math relation:
 *   T_world_grasp = T_world_part * T_part_grasp
 *   T_part_grasp  = inv(T_world_part) * T_world_grasp
 */

export class PoseMath {
  private static tempMat1 = new THREE.Matrix4();
  private static tempMat2 = new THREE.Matrix4();
  private static tempInv = new THREE.Matrix4();

  /**
   * Converts a Three.js Matrix4 to a serializable Pose (position xyz in mm + quaternion xyzw)
   */
  public static matrixToPose(matrix: THREE.Matrix4): Pose {
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    matrix.decompose(pos, quat, scale);

    return {
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
  }

  /**
   * Converts a Pose to a Three.js Matrix4
   */
  public static poseToMatrix(pose: Pose): THREE.Matrix4 {
    const pos = new THREE.Vector3(pose.position.x, pose.position.y, pose.position.z);
    const quat = new THREE.Quaternion(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
    );
    return new THREE.Matrix4().compose(pos, quat, new THREE.Vector3(1, 1, 1));
  }

  /**
   * Converts a quaternion to Euler Roll, Pitch, Yaw in degrees (XYZ order)
   */
  public static quaternionToEulerRPY(quat: THREE.Quaternion): EulerAngles {
    const euler = new THREE.Euler().setFromQuaternion(quat, 'XYZ');
    return {
      roll: Number(THREE.MathUtils.radToDeg(euler.x).toFixed(2)),
      pitch: Number(THREE.MathUtils.radToDeg(euler.y).toFixed(2)),
      yaw: Number(THREE.MathUtils.radToDeg(euler.z).toFixed(2))
    };
  }

  /**
   * Converts Euler Roll, Pitch, Yaw in degrees to a Three.js Quaternion
   */
  public static eulerRPYToQuaternion(rpy: EulerAngles): THREE.Quaternion {
    const euler = new THREE.Euler(
      THREE.MathUtils.degToRad(rpy.roll),
      THREE.MathUtils.degToRad(rpy.pitch),
      THREE.MathUtils.degToRad(rpy.yaw),
      'XYZ'
    );
    return new THREE.Quaternion().setFromEuler(euler);
  }

  /**
   * Computes T_part_grasp = inv(T_world_part) * T_world_grasp
   */
  public static computePartGraspMatrix(
    T_world_part: THREE.Matrix4,
    T_world_grasp: THREE.Matrix4
  ): THREE.Matrix4 {
    this.tempInv.copy(T_world_part).invert();
    return new THREE.Matrix4().multiplyMatrices(this.tempInv, T_world_grasp);
  }

  /**
   * Computes T_part_grasp as a Pose
   */
  public static computePartGraspPose(
    T_world_part: THREE.Matrix4,
    T_world_grasp: THREE.Matrix4
  ): Pose {
    const mat = this.computePartGraspMatrix(T_world_part, T_world_grasp);
    return this.matrixToPose(mat);
  }

  /**
   * Computes T_world_grasp = T_world_part * T_part_grasp
   */
  public static computeWorldGraspMatrix(
    T_world_part: THREE.Matrix4,
    T_part_grasp: THREE.Matrix4
  ): THREE.Matrix4 {
    return new THREE.Matrix4().multiplyMatrices(T_world_part, T_part_grasp);
  }

  /**
   * Computes world position and quaternion from T_world_part and T_part_grasp (Pose)
   */
  public static computeWorldGraspTransform(
    T_world_part: THREE.Matrix4,
    graspPose: Pose
  ): { position: THREE.Vector3; quaternion: THREE.Quaternion } {
    const T_part_grasp = this.poseToMatrix(graspPose);
    const T_world_grasp = this.computeWorldGraspMatrix(T_world_part, T_part_grasp);

    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    T_world_grasp.decompose(pos, quat, scale);

    return { position: pos, quaternion: quat };
  }

  /**
   * Transforms a 3D point from World space to CAD Part local frame
   */
  public static worldPointToPartFrame(
    worldPoint: THREE.Vector3,
    T_world_part: THREE.Matrix4
  ): Vector3D {
    this.tempInv.copy(T_world_part).invert();
    const pt = worldPoint.clone().applyMatrix4(this.tempInv);
    return {
      x: Number(pt.x.toFixed(3)),
      y: Number(pt.y.toFixed(3)),
      z: Number(pt.z.toFixed(3))
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
