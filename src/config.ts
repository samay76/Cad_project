/**
 * Configuration for Gripper and Cobot Pick-Point Tool
 */

export const GRIPPER_CONFIG = {
  // Folder containing gripper assets
  folderPath: '/home/human/nferent_cad/robotiq_2f85',

  // Served URL path in Vite dev server
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
 * Selectable grippers. All share the same TCP convention: URDF root = tool flange, +Z = approach,
 * fingers close along X, TCP at (0, 0, tcpOffset) from the flange.
 */
export type GripperId = '2f85' | 'hande';

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
  hande: {
    id: 'hande',
    label: 'Robotiq Hand-E',
    baseUrl: '/grippers/robotiq_hande/',
    urdfFile: 'robotiq_hande.urdf',
    packages: { robotiq_hande_description: '/grippers/robotiq_hande' },
    maxOpeningWidth: 50.0,
    // flange -> fingertip = 11 (coupler) + 99 (body) + 46.5 (finger) = 156.5 mm; TCP 10 mm into the finger faces
    defaultTcpOffset: 146.5,
    kinematics: 'prismatic',
    leftTipLink: 'robotiq_hande_left_finger',
    rightTipLink: 'robotiq_hande_right_finger',
    leftFingerJoint: 'robotiq_hande_left_finger_joint',
    rightFingerJoint: 'robotiq_hande_right_finger_joint',
    fingerTravel: 0.025,
    // The finger visual mesh includes the slider inside the body; use the gripping block only
    // (bounds of finger_collision.dae; the right finger is rotated 180 deg about Z).
    proxyBoxes: {
      robotiq_hande_left_finger: { min: [0.0, -0.0107, 0.006], max: [0.011, 0.0105, 0.0476] },
      robotiq_hande_right_finger: { min: [-0.011, -0.0105, 0.006], max: [0.0, 0.0107, 0.0476] }
    }
  }
};

const GRIPPER_STORAGE_KEY = 'nferent_cad_gripper';

export const getStoredGripperId = (): GripperId => {
  try {
    const id = localStorage.getItem(GRIPPER_STORAGE_KEY);
    if (id && id in GRIPPERS) return id as GripperId;
  } catch {
    // ignore storage errors
  }
  return '2f85';
};

/** Makes `spec` the active gripper: GRIPPER_CONFIG limits follow it (used across the app). */
export const applyGripperSpec = (spec: GripperSpec) => {
  GRIPPER_CONFIG.baseUrl = spec.baseUrl;
  GRIPPER_CONFIG.urdfFile = spec.urdfFile;
  GRIPPER_CONFIG.maxOpeningWidth = spec.maxOpeningWidth;
  GRIPPER_CONFIG.defaultOpeningWidth = spec.maxOpeningWidth;
  GRIPPER_CONFIG.defaultTcpOffset = spec.defaultTcpOffset;
  try {
    localStorage.setItem(GRIPPER_STORAGE_KEY, spec.id);
  } catch {
    // ignore storage errors
  }
};
