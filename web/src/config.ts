/**
 * Configuration for Gripper and Cobot Pick-Point Tool
 */

export const GRIPPER_CONFIG = {
  // URL path the gripper URDF is served from (web/public/gripper)
  baseUrl: '/gripper/',

  // URDF specification file
  urdfFile: 'robotiq_2f_85.urdf',

  /**
   * UNIT CONVERSION:
   * The Robotiq 2F-85 URDF description and meshes are modeled in standard SI units (METERS).
   * The CAD models and our 3D viewer coordinate system operate in MILLIMETERS.
   * `unitScale` converts meters to millimeters (1.0 meter = 1000.0 millimeters).
   * The URDF robot root is scaled by this factor in a single place.
   */
  unitScale: 1000.0,

  // Finger opening limits (Robotiq 2F-85 physical limits in mm)
  minOpeningWidth: 0.0,
  maxOpeningWidth: 85.0,
  defaultOpeningWidth: 85.0,

  // Default TCP (Tool Center Point) offset from mounting flange in mm
  defaultTcpOffset: 145.0,

  // Default approach retraction distance for pre-grasp in mm
  defaultPreGraspDistance: 100.0,

  // Colors
  gripperBodyColor: 0x334155, // Dark slate
  metalAccentColor: 0x94a3b8, // Light metal accent
  collisionColor: 0xef4444    // Red warning on collision
};

/**
 * The cell's gripper (Robotiq 2F-85). TCP convention: URDF root = tool flange, +Z = approach,
 * fingers close along X, TCP at (0, 0, tcpOffset) from the flange.
 */
export type GripperId = '2f85';

export interface GripperSpec {
  id: GripperId;
  label: string;
  baseUrl: string; // served folder containing the URDF
  urdfFile: string;
  packages: Record<string, string>; // package:// name -> served folder
  maxOpeningWidth: number; // mm between the finger faces when fully open
  defaultTcpOffset: number; // mm from the flange along +Z
  /** 'linkage' = Robotiq 2F-85 4-bar fingers; 'prismatic' = parallel sliding fingers. */
  kinematics: 'linkage' | 'prismatic';
  leftTipLink: string;
  rightTipLink: string;
  /** prismatic only: finger joints and travel per finger (URDF units, m) at full opening. */
  leftFingerJoint?: string;
  rightFingerJoint?: string;
  fingerTravel?: number;
  /** Optional collision proxy boxes in link frame (m), instead of the visual mesh bounds. */
  proxyBoxes?: Record<string, { min: [number, number, number]; max: [number, number, number] }>;
}

export const GRIPPERS: Record<GripperId, GripperSpec> = {
  '2f85': {
    id: '2f85',
    label: 'Robotiq 2F-85',
    baseUrl: '/gripper/',
    urdfFile: 'robotiq_2f_85.urdf',
    packages: { robotiq_description: '/gripper' },
    maxOpeningWidth: 85.0,
    defaultTcpOffset: 145.0,
    kinematics: 'linkage',
    leftTipLink: 'robotiq_85_left_finger_tip_link',
    rightTipLink: 'robotiq_85_right_finger_tip_link'
  },
};

/** The cell's robot arm (cell/robots.py). */
export type RobotId = 'ec66';
export const ROBOT_ID: RobotId = 'ec66';
export const GRIPPER_ID: GripperId = '2f85';

/** Makes `spec` the active gripper: GRIPPER_CONFIG limits follow it (used across the app). */
export const applyGripperSpec = (spec: GripperSpec) => {
  GRIPPER_CONFIG.baseUrl = spec.baseUrl;
  GRIPPER_CONFIG.urdfFile = spec.urdfFile;
  GRIPPER_CONFIG.maxOpeningWidth = spec.maxOpeningWidth;
  GRIPPER_CONFIG.defaultOpeningWidth = spec.maxOpeningWidth;
  GRIPPER_CONFIG.defaultTcpOffset = spec.defaultTcpOffset;
};
