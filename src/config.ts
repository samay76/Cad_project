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
