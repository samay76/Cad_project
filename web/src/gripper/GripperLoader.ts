import * as THREE from 'three';
import URDFLoader, { type URDFRobot } from 'urdf-loader';
import { GRIPPER_CONFIG, type GripperSpec } from '../config';
import { GripperModel } from './GripperModel';

export class GripperLoader {
  /**
   * Loads a gripper URDF (see GRIPPERS in config.ts) using URDFLoader
   */
  public static async loadGripper(spec: GripperSpec): Promise<GripperModel> {
    const urdfUrl = `${spec.baseUrl}${spec.urdfFile}`;

    // Resolve only when every file (URDF + all meshes) has loaded: collision proxies are built from
    // the visual meshes, so a finger mesh arriving late would leave that finger without contact checks.
    const manager = new THREE.LoadingManager();
    const allLoaded = new Promise<void>((resolve) => {
      manager.onLoad = () => resolve();
    });
    const loader = new URDFLoader(manager);
    loader.packages = { ...spec.packages };

    let robot: URDFRobot;
    try {
      robot = await loader.loadAsync(urdfUrl);
      await Promise.race([allLoaded, new Promise((r) => setTimeout(r, 30000))]);
    } catch (err: any) {
      throw new Error(`Failed to load ${spec.label} URDF from ${urdfUrl}: ${err?.message || err}`);
    }

    robot.scale.set(
      GRIPPER_CONFIG.unitScale,
      GRIPPER_CONFIG.unitScale,
      GRIPPER_CONFIG.unitScale
    );

    return new GripperModel(robot, spec);
  }
}
