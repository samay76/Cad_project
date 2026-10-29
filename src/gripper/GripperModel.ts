import * as THREE from 'three';
import { GRIPPER_CONFIG } from '../config';

export interface GripperMeshes {
  base: THREE.Mesh;
  baseCoupling?: THREE.Mesh;
  collar?: THREE.Mesh;
  leftDriver: THREE.Mesh;
  leftCoupler: THREE.Mesh;
  leftFollower: THREE.Mesh;
  leftSpringLink: THREE.Mesh;
  leftTongue?: THREE.Mesh;
  rightDriver: THREE.Mesh;
  rightCoupler: THREE.Mesh;
  rightFollower: THREE.Mesh;
  rightSpringLink: THREE.Mesh;
  rightTongue?: THREE.Mesh;
  allMeshes: THREE.Mesh[];
}

export class GripperModel {
  // Master root group located at the TCP (Tool Center Point)
  public root: THREE.Group;

  // Subgroup offset by tcpOffset along -Z
  private bodyGroup: THREE.Group;

  // Finger kinematic groups
  private leftFingerGroup: THREE.Group;
  private rightFingerGroup: THREE.Group;

  // Visual frame helpers
  private tcpAxesHelper: THREE.AxesHelper;
  private approachArrow: THREE.ArrowHelper;
  private tcpCenterSphere: THREE.Mesh;

  // State
  private openingWidth = GRIPPER_CONFIG.defaultOpeningWidth;
  private tcpOffset = GRIPPER_CONFIG.defaultTcpOffset;
  private isColliding = false;

  private allMeshes: THREE.Mesh[] = [];
  private originalMaterials: Map<THREE.Mesh, THREE.Material | THREE.Material[]> = new Map();
  private collisionMaterial: THREE.MeshStandardMaterial;

  constructor(meshes: GripperMeshes) {
    this.root = new THREE.Group();
    this.root.name = 'Robotiq_2F85_TCP';

    this.bodyGroup = new THREE.Group();
    this.bodyGroup.name = 'GripperBody';
    this.root.add(this.bodyGroup);

    this.allMeshes = meshes.allMeshes;

    // Cache original materials
    for (const mesh of this.allMeshes) {
      this.originalMaterials.set(mesh, mesh.material);
    }

    this.collisionMaterial = new THREE.MeshStandardMaterial({
      color: GRIPPER_CONFIG.collisionColor,
      emissive: 0xb91c1c,
      emissiveIntensity: 0.5,
      roughness: 0.35,
      metalness: 0.1
    });

    // 1. Add base housing to bodyGroup
    this.bodyGroup.add(meshes.base);
    if (meshes.baseCoupling) this.bodyGroup.add(meshes.baseCoupling);
    if (meshes.collar) this.bodyGroup.add(meshes.collar);

    // 2. Setup Left Finger Group
    this.leftFingerGroup = new THREE.Group();
    this.leftFingerGroup.name = 'LeftFinger';
    this.leftFingerGroup.add(meshes.leftDriver);
    this.leftFingerGroup.add(meshes.leftCoupler);
    this.leftFingerGroup.add(meshes.leftFollower);
    this.leftFingerGroup.add(meshes.leftSpringLink);
    if (meshes.leftTongue) this.leftFingerGroup.add(meshes.leftTongue);
    this.bodyGroup.add(this.leftFingerGroup);

    // 3. Setup Right Finger Group
    this.rightFingerGroup = new THREE.Group();
    this.rightFingerGroup.name = 'RightFinger';
    this.rightFingerGroup.add(meshes.rightDriver);
    this.rightFingerGroup.add(meshes.rightCoupler);
    this.rightFingerGroup.add(meshes.rightFollower);
    this.rightFingerGroup.add(meshes.rightSpringLink);
    if (meshes.rightTongue) this.rightFingerGroup.add(meshes.rightTongue);
    this.bodyGroup.add(this.rightFingerGroup);

    // 4. Setup TCP Frame Visual Helpers (RGB axes + Approach Arrow)
    this.tcpAxesHelper = new THREE.AxesHelper(30);
    this.tcpAxesHelper.renderOrder = 10;
    this.root.add(this.tcpAxesHelper);

    // Approach arrow pointing along +Z (outward along approach direction)
    const approachDir = new THREE.Vector3(0, 0, 1);
    this.approachArrow = new THREE.ArrowHelper(
      approachDir,
      new THREE.Vector3(0, 0, 0),
      45,
      0x0284c7, // vibrant cyan blue
      10,
      5
    );
    this.approachArrow.renderOrder = 11;
    this.root.add(this.approachArrow);

    // Small sphere at TCP center
    const sphereGeom = new THREE.SphereGeometry(1.5, 16, 16);
    const sphereMat = new THREE.MeshBasicMaterial({ color: 0x0284c7 });
    this.tcpCenterSphere = new THREE.Mesh(sphereGeom, sphereMat);
    this.tcpCenterSphere.renderOrder = 11;
    this.root.add(this.tcpCenterSphere);

    // Initial positioning
    this.updateTcpOffset();
    this.setOpeningWidth(this.openingWidth);
  }

  /**
   * Sets the TCP offset in mm from mounting flange to fingertip center
   */
  public setTcpOffset(offsetMm: number) {
    this.tcpOffset = Math.max(50, offsetMm);
    this.updateTcpOffset();
  }

  public getTcpOffset(): number {
    return this.tcpOffset;
  }

  private updateTcpOffset() {
    // Offset the gripper body along -Z so the TCP (0,0,0) is at the fingertip center
    this.bodyGroup.position.set(0, 0, -this.tcpOffset);
  }

  /**
   * Adjusts finger opening width in millimeters [0 to 85 mm]
   */
  public setOpeningWidth(widthMm: number) {
    this.openingWidth = Math.max(
      GRIPPER_CONFIG.minOpeningWidth,
      Math.min(GRIPPER_CONFIG.maxOpeningWidth, widthMm)
    );

    // Normalizing ratio: 0 (fully open 85mm) to 1 (fully closed 0mm)
    const closeRatio = (GRIPPER_CONFIG.maxOpeningWidth - this.openingWidth) / GRIPPER_CONFIG.maxOpeningWidth;

    // Lateral displacement of fingers (max delta ~42.5mm per finger)
    const dx = closeRatio * 42.5;

    // Dynamic 4-bar linkage inward rotation
    const rot = closeRatio * 0.72; // ~41 degrees

    // Left finger moves in +X direction and rotates
    this.leftFingerGroup.position.x = dx;
    this.leftFingerGroup.rotation.y = -rot * 0.5;

    // Right finger moves in -X direction and rotates symmetrically
    this.rightFingerGroup.position.x = -dx;
    this.rightFingerGroup.rotation.y = rot * 0.5;
  }

  public getOpeningWidth(): number {
    return this.openingWidth;
  }

  /**
   * Toggles collision highlight
   */
  public setCollisionHighlight(colliding: boolean) {
    if (this.isColliding === colliding) return;
    this.isColliding = colliding;

    for (const mesh of this.allMeshes) {
      if (colliding) {
        mesh.material = this.collisionMaterial;
      } else {
        const orig = this.originalMaterials.get(mesh);
        if (orig) mesh.material = orig;
      }
    }
  }

  public getIsColliding(): boolean {
    return this.isColliding;
  }

  public getAllMeshes(): THREE.Mesh[] {
    return this.allMeshes;
  }

  public setVisible(visible: boolean) {
    this.root.visible = visible;
  }

  public isVisible(): boolean {
    return this.root.visible;
  }

  public dispose() {
    this.tcpAxesHelper.dispose();
    this.approachArrow.dispose();
    this.tcpCenterSphere.geometry.dispose();
    (this.tcpCenterSphere.material as THREE.Material).dispose();
    this.collisionMaterial.dispose();
  }
}
