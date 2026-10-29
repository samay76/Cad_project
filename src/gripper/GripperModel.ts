import * as THREE from 'three';
import type { URDFRobot } from 'urdf-loader';
import { GRIPPER_CONFIG } from '../config';

export class GripperModel {
  // Master root group located at the TCP (Tool Center Point)
  public root: THREE.Group;

  // Subgroup offset by tcpOffset along -Z
  private bodyGroup: THREE.Group;

  // URDF Robot instance
  private robot: URDFRobot;

  // Visual frame helpers
  private tcpAxesHelper: THREE.AxesHelper;
  private approachArrow: THREE.ArrowHelper;
  private tcpCenterSphere: THREE.Mesh;

  // State
  private openingWidth = GRIPPER_CONFIG.defaultOpeningWidth;
  private tcpOffset = GRIPPER_CONFIG.defaultTcpOffset;
  private isColliding = false;

  private allMeshes: THREE.Mesh[] = [];
  private fingerTipMeshes: THREE.Mesh[] = [];
  private bodyMeshes: THREE.Mesh[] = [];
  private originalMaterials: Map<THREE.Mesh, THREE.Material | THREE.Material[]> = new Map();
  private collisionMaterial: THREE.MeshStandardMaterial;

  constructor(robot: URDFRobot) {
    this.root = new THREE.Group();
    this.root.name = 'Robotiq_2F85_TCP';

    this.bodyGroup = new THREE.Group();
    this.bodyGroup.name = 'GripperBody';
    this.root.add(this.bodyGroup);

    this.robot = robot;
    this.bodyGroup.add(this.robot);

    // Cache original materials and collect all meshes for raycasting & collision
    this.robot.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        this.allMeshes.push(child);
        this.originalMaterials.set(child, child.material);

        // Ensure materials display nicely under Three.js lighting
        if (child.material instanceof THREE.MeshStandardMaterial || child.material instanceof THREE.MeshPhongMaterial) {
          child.material.side = THREE.DoubleSide;
        }
      }
    });

    // Categorize fingertip contact pads vs body meshes
    const leftTip = this.robot.links['robotiq_85_left_finger_tip_link'];
    const rightTip = this.robot.links['robotiq_85_right_finger_tip_link'];
    if (leftTip) {
      leftTip.traverse((c) => {
        if (c instanceof THREE.Mesh) this.fingerTipMeshes.push(c);
      });
    }
    if (rightTip) {
      rightTip.traverse((c) => {
        if (c instanceof THREE.Mesh) this.fingerTipMeshes.push(c);
      });
    }

    this.allMeshes.forEach((m) => {
      if (!this.fingerTipMeshes.includes(m)) {
        this.bodyMeshes.push(m);
      }
    });

    this.collisionMaterial = new THREE.MeshStandardMaterial({
      color: GRIPPER_CONFIG.collisionColor,
      emissive: 0xb91c1c,
      emissiveIntensity: 0.5,
      roughness: 0.35,
      metalness: 0.1
    });

    // 1. Setup TCP Frame Visual Helpers (RGB axes + Approach Arrow + Sphere)
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

    // Initial positioning & finger joint configuration
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
    // In URDF, the mounting flange is at (0,0,0) and fingers extend in +Z.
    // Offsetting the body along -Z aligns fingertips at the TCP (0, 0, 0)
    // with +Z pointing forward out of the gripper.
    this.bodyGroup.position.set(0, 0, -this.tcpOffset);
  }

  /**
   * Adjusts finger opening width in millimeters [0 to 85 mm] using URDF kinematic joints
   */
  public setOpeningWidth(widthMm: number) {
    this.openingWidth = Math.max(
      GRIPPER_CONFIG.minOpeningWidth,
      Math.min(GRIPPER_CONFIG.maxOpeningWidth, widthMm)
    );

    // In Robotiq 2F-85 URDF:
    // angle 0.0 rad = fully open (85 mm)
    // angle 0.8 rad = fully closed (0 mm)
    const angle = (1.0 - (this.openingWidth / GRIPPER_CONFIG.maxOpeningWidth)) * 0.8;

    // Set driving joint
    if (this.robot.joints['robotiq_85_left_knuckle_joint']) {
      this.robot.setJointValue('robotiq_85_left_knuckle_joint', angle);
    }
    // Set/confirm mimic joints
    if (this.robot.joints['robotiq_85_right_knuckle_joint']) {
      this.robot.setJointValue('robotiq_85_right_knuckle_joint', -angle);
    }
    if (this.robot.joints['robotiq_85_left_inner_knuckle_joint']) {
      this.robot.setJointValue('robotiq_85_left_inner_knuckle_joint', angle);
    }
    if (this.robot.joints['robotiq_85_right_inner_knuckle_joint']) {
      this.robot.setJointValue('robotiq_85_right_inner_knuckle_joint', -angle);
    }
    if (this.robot.joints['robotiq_85_left_finger_tip_joint']) {
      this.robot.setJointValue('robotiq_85_left_finger_tip_joint', -angle);
    }
    if (this.robot.joints['robotiq_85_right_finger_tip_joint']) {
      this.robot.setJointValue('robotiq_85_right_finger_tip_joint', angle);
    }
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

  public getFingerTipMeshes(): THREE.Mesh[] {
    return this.fingerTipMeshes;
  }

  public getBodyMeshes(): THREE.Mesh[] {
    return this.bodyMeshes;
  }

  public getRobot(): URDFRobot {
    return this.robot;
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
