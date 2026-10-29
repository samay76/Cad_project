/**
 * Core Types for CAD-based Cobot Pick-Point Viewer
 */

export interface Vector3D {
  x: number;
  y: number;
  z: number;
}

export interface QuaternionD {
  x: number;
  y: number;
  z: number;
  w: number;
}

/**
 * Pose type: position xyz + quaternion (for cobot grasp points)
 * Defined per requirements for downstream pick-and-place cobot tool integration.
 */
export interface Pose {
  position: Vector3D;
  orientation: QuaternionD;
}

export interface BoundingBoxInfo {
  min: Vector3D;
  max: Vector3D;
  size: Vector3D;    // Size in mm (X, Y, Z)
  center: Vector3D;
}

export interface PartInfo {
  id: string;
  index: number;
  name: string;
  triangleCount: number;
  vertexCount: number;
  visible: boolean;
  color?: string;
  meshIndex: number;
}

export interface CADModelMetadata {
  fileName: string;
  fileSize: number;
  fileType: 'step' | 'stl' | 'obj' | 'unknown';
  partsCount: number;
  triangleCount: number;
  vertexCount: number;
  boundingBox: BoundingBoxInfo;
}

export interface ClickedPointInfo {
  // Point in the model's native coordinate frame (mm, 3 decimals)
  point: Vector3D;
  // Face normal unit vector in the model's coordinate frame
  normal: Vector3D;
  // World space equivalents
  worldPoint: Vector3D;
  worldNormal: Vector3D;
  // Associated part details
  partIndex: number;
  partName: string;
}

export interface SelectionState {
  selectedPartId: string | null;
  selectedPartIndex: number | null;
  selectedPartName: string | null;
  clickedPoint: ClickedPointInfo | null;
}

export interface EulerAngles {
  roll: number;  // degrees
  pitch: number; // degrees
  yaw: number;   // degrees
}

export interface SavedGrasp {
  id: string;
  name: string;
  tcpPose: Pose; // T_part_grasp: Pose in part coordinate frame (mm + quaternion)
  rpy: EulerAngles; // Roll, Pitch, Yaw in degrees for readability
  openingWidth: number; // mm
  preGraspDistance: number; // mm
  tcpOffset: number; // mm
  createdAt: number;
  partName?: string;
}

export type TestPhase =
  | 'idle'
  | 'pre-grasp'
  | 'approach'
  | 'close'
  | 'lift'
  | 'transfer'
  | 'place'
  | 'release'
  | 'retract';

export interface GripperState {
  loaded: boolean;
  visible: boolean;
  openingWidth: number; // mm
  contactWidth?: number; // detected workpiece grasp thickness in mm
  tcpOffset: number; // mm
  preGraspDistance: number; // mm
  pose: Pose; // T_part_grasp in mm
  rpy: EulerAngles;
  hasCollision: boolean;
  isTesting: boolean;
  testPhase: TestPhase;
}
