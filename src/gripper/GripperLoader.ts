import URDFLoader, { type URDFRobot } from 'urdf-loader';
import { GRIPPER_CONFIG } from '../config';
import { GripperModel } from './GripperModel';

export class GripperLoader {
  /**
   * Loads the Robotiq 2F-85 URDF model using URDFLoader
   */
  public static async loadGripper(): Promise<GripperModel> {
    const baseUrl = GRIPPER_CONFIG.baseUrl;
    const urdfFileName = GRIPPER_CONFIG.urdfFile || 'robotiq_2f_85.urdf';
    const urdfUrl = `${baseUrl}${urdfFileName}`;

    const loader = new URDFLoader();
    // Resolve package://robotiq_description/ to /gripper
    loader.packages = {
      robotiq_description: baseUrl.replace(/\/$/, '')
    };

    let robot: URDFRobot;
    try {
      robot = await loader.loadAsync(urdfUrl);
    } catch (err: any) {
      throw new Error(`Failed to load Robotiq 2F-85 URDF from ${urdfUrl}: ${err?.message || err}`);
    }

    /**
     * UNIT CONVERSION IN A SINGLE PLACE:
     * The Robotiq 2F-85 URDF model is defined in standard SI units (METERS).
     * The CAD viewer operates in MILLIMETERS.
     * We scale the entire URDF robot hierarchy by GRIPPER_CONFIG.unitScale (1000.0).
     */
    robot.scale.set(
      GRIPPER_CONFIG.unitScale,
      GRIPPER_CONFIG.unitScale,
      GRIPPER_CONFIG.unitScale
    );

    return new GripperModel(robot);
  }
}
