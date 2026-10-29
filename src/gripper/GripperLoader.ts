import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { GRIPPER_CONFIG } from '../config';
import { GripperModel, type GripperMeshes } from './GripperModel';

export class GripperLoader {
  private static loader = new STLLoader();

  /**
   * Inspects the gripper assets folder and loads the Robotiq 2F-85 model
   */
  public static async loadGripper(): Promise<GripperModel> {
    const baseUrl = GRIPPER_CONFIG.baseUrl;

    // Verify folder accessibility
    try {
      const checkResp = await fetch(`${baseUrl}2f85.xml`);
      if (!checkResp.ok) {
        throw new Error(`Gripper description file not found at ${baseUrl}2f85.xml (HTTP ${checkResp.status})`);
      }
    } catch (err: any) {
      throw new Error(`Failed to access gripper folder at ${baseUrl}. Ensure Vite is configured to serve the gripper directory: ${err?.message}`);
    }

    // Material definitions (Dark industrial slate body + metal guides)
    const bodyMaterial = new THREE.MeshStandardMaterial({
      color: GRIPPER_CONFIG.gripperBodyColor,
      roughness: 0.35,
      metalness: 0.25,
      side: THREE.DoubleSide
    });

    const metalMaterial = new THREE.MeshStandardMaterial({
      color: GRIPPER_CONFIG.metalAccentColor,
      roughness: 0.25,
      metalness: 0.7,
      side: THREE.DoubleSide
    });

    // Helper to fetch, parse, and convert STL mesh to MILLIMETERS
    const loadMesh = async (fileName: string, mat: THREE.Material): Promise<THREE.Mesh> => {
      const url = `${baseUrl}assets/${fileName}`;
      const res = await fetch(url);
      if (!res.ok) {
        throw new Error(`Missing gripper mesh file: ${fileName} at ${url}`);
      }
      const buffer = await res.arrayBuffer();
      const geom = this.loader.parse(buffer);

      /**
       * UNIT CONVERSION IN A SINGLE PLACE:
       * The Robotiq 2F-85 meshes are defined in meters.
       * Multiplying vertices by GRIPPER_CONFIG.unitScale (1000.0) converts them to millimeters.
       */
      geom.scale(GRIPPER_CONFIG.unitScale, GRIPPER_CONFIG.unitScale, GRIPPER_CONFIG.unitScale);
      geom.computeVertexNormals();

      const mesh = new THREE.Mesh(geom, mat);
      mesh.name = fileName.replace(/\.stl$/i, '');
      mesh.castShadow = false;
      mesh.receiveShadow = false;
      return mesh;
    };

    // Load STL meshes in parallel
    const [
      baseMesh,
      baseCouplingMesh,
      collarMesh,
      driverLeft,
      couplerLeft,
      followerLeft,
      springLeft,
      tongueLeft,
      driverRight,
      couplerRight,
      followerRight,
      springRight,
      tongueRight
    ] = await Promise.all([
      loadMesh('base.stl', bodyMaterial),
      loadMesh('base_coupling.stl', bodyMaterial),
      loadMesh('c-a01-85-open.stl', metalMaterial),
      loadMesh('driver.stl', metalMaterial),
      loadMesh('coupler.stl', bodyMaterial),
      loadMesh('follower.stl', bodyMaterial),
      loadMesh('spring_link.stl', bodyMaterial),
      loadMesh('tongue.stl', metalMaterial),
      loadMesh('driver.stl', metalMaterial),
      loadMesh('coupler.stl', bodyMaterial),
      loadMesh('follower.stl', bodyMaterial),
      loadMesh('spring_link.stl', bodyMaterial),
      loadMesh('tongue.stl', metalMaterial)
    ]);

    // Position Base components (Z-up, in millimeters)
    baseMesh.position.set(0, 0, 10.8);
    baseCouplingMesh.position.set(0, 0, 4.0);
    collarMesh.position.set(0, 0, 10.8);

    // Left 4-bar linkage positions (in mm: pos * 1000)
    driverLeft.position.set(-30.60, 4.75, 65.70);
    couplerLeft.position.set(-31.42, 4.53, 55.50);
    followerLeft.position.set(-38.21, -4.25, 75.17);
    springLeft.position.set(-12.70, -12.00, 72.22);
    tongueLeft.position.set(-38.21, -4.25, 75.17);

    // Right 4-bar linkage positions (symmetrically mirrored across X)
    driverRight.position.set(30.60, -4.75, 65.70);
    driverRight.rotation.z = Math.PI;

    couplerRight.position.set(31.42, -4.53, 55.50);
    couplerRight.rotation.z = Math.PI;

    followerRight.position.set(38.21, 4.25, 75.17);
    followerRight.rotation.z = Math.PI;

    springRight.position.set(12.70, 12.00, 72.22);
    springRight.rotation.z = Math.PI;

    tongueRight.position.set(38.21, 4.25, 75.17);
    tongueRight.rotation.z = Math.PI;

    const allMeshes: THREE.Mesh[] = [
      baseMesh,
      baseCouplingMesh,
      collarMesh,
      driverLeft,
      couplerLeft,
      followerLeft,
      springLeft,
      tongueLeft,
      driverRight,
      couplerRight,
      followerRight,
      springRight,
      tongueRight
    ];

    const meshes: GripperMeshes = {
      base: baseMesh,
      baseCoupling: baseCouplingMesh,
      collar: collarMesh,
      leftDriver: driverLeft,
      leftCoupler: couplerLeft,
      leftFollower: followerLeft,
      leftSpringLink: springLeft,
      leftTongue: tongueLeft,
      rightDriver: driverRight,
      rightCoupler: couplerRight,
      rightFollower: followerRight,
      rightSpringLink: springRight,
      rightTongue: tongueRight,
      allMeshes
    };

    return new GripperModel(meshes);
  }
}
