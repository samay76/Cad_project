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
    loader.packages = {
      robotiq_description: baseUrl.replace(/\/$/, '')
    };

    let robot: URDFRobot;
    try {
      robot = await loader.loadAsync(urdfUrl);
    } catch (err: any) {
      throw new Error(`Failed to load Robotiq 2F-85 URDF from ${urdfUrl}: ${err?.message || err}`);
    }

    robot.scale.set(
      GRIPPER_CONFIG.unitScale,
      GRIPPER_CONFIG.unitScale,
      GRIPPER_CONFIG.unitScale
    );

    // Wait until visual meshes have finished loading into links
    await new Promise<void>((resolve) => {
      let attempts = 0;
      const check = () => {
        let meshCount = 0;
        robot.traverse((c) => {
          if ((c as any).isMesh) meshCount++;
        });
        if (meshCount >= 9 || attempts > 100) {
          resolve();
        } else {
          attempts++;
          setTimeout(check, 30);
        }
      };
      check();
    });

    return new GripperModel(robot);
  }
}
