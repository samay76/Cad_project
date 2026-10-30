import * as THREE from 'three';
import type { URDFRobot } from 'urdf-loader';
import { MeshBVH } from 'three-mesh-bvh';
import { GRIPPER_CONFIG, type GripperSpec } from '../config';

export class GripperModel {
  // Master root group located at the TCP (Tool Center Point)
  public root: THREE.Group;

  // Subgroup offset by tcpOffset along -Z
  private bodyGroup: THREE.Group;

  // URDF Robot instance
  private robot: URDFRobot;
  public readonly spec: GripperSpec;

  // Visual frame helpers
  private tcpAxesHelper: THREE.AxesHelper;
  private approachArrow: THREE.ArrowHelper;
  private tcpCenterSphere: THREE.Mesh;

  // State
  private openingWidth = GRIPPER_CONFIG.defaultOpeningWidth;
  private tcpOffset = GRIPPER_CONFIG.defaultTcpOffset;
  private isColliding = false;

  // Visual meshes
  private allMeshes: THREE.Mesh[] = [];
  private fingerTipMeshes: THREE.Mesh[] = [];
  private leftFingerTipMeshes: THREE.Mesh[] = [];
  private rightFingerTipMeshes: THREE.Mesh[] = [];
  private bodyMeshes: THREE.Mesh[] = [];
  private originalMaterials: Map<THREE.Mesh, THREE.Material | THREE.Material[]> = new Map();
  private collisionMaterial: THREE.MeshStandardMaterial;

  // Low-poly collision proxies (12 triangles each, 108 triangles total)
  private bodyProxies: THREE.Mesh[] = [];
  private leftTipProxy: THREE.Mesh | null = null;
  private rightTipProxy: THREE.Mesh | null = null;
  private allProxies: THREE.Mesh[] = [];

  // Reusable scratch objects
  private _scratchBox = new THREE.Box3();
  private _scratchVec = new THREE.Vector3();

  constructor(robot: URDFRobot, spec: GripperSpec) {
    this.spec = spec;
    this.openingWidth = spec.maxOpeningWidth;
    this.tcpOffset = spec.defaultTcpOffset;
    this.root = new THREE.Group();
    this.root.name = `${spec.id}_TCP`;

    this.bodyGroup = new THREE.Group();
    this.bodyGroup.name = 'GripperBody';
    this.root.add(this.bodyGroup);

    this.robot = robot;
    this.bodyGroup.add(this.robot);

    this.collisionMaterial = new THREE.MeshStandardMaterial({
      color: GRIPPER_CONFIG.collisionColor,
      emissive: 0xb91c1c,
      emissiveIntensity: 0.5,
      roughness: 0.35,
      metalness: 0.1
    });

    // 1. Setup visual mesh caches and material maps
    this.setupVisualMeshes();

    // 2. Setup low-poly collision proxies parented to URDF links
    this.setupCollisionProxies();

    // 3. Setup TCP Frame Visual Helpers (RGB axes + Approach Arrow + Sphere)
    this.tcpAxesHelper = new THREE.AxesHelper(30);
    this.tcpAxesHelper.renderOrder = 10;
    this.root.add(this.tcpAxesHelper);

    // Approach arrow pointing along +Z (outward along approach direction)
    const approachDir = new THREE.Vector3(0, 0, 1);
    this.approachArrow = new THREE.ArrowHelper(
      approachDir,
      new THREE.Vector3(0, 0, 0),
      45,
      0x0284c7,
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

  private setupVisualMeshes() {
    this.allMeshes = [];
    this.fingerTipMeshes = [];
    this.leftFingerTipMeshes = [];
    this.rightFingerTipMeshes = [];
    this.bodyMeshes = [];
    this.originalMaterials.clear();

    // Cache original materials and collect all meshes for rendering
    this.robot.traverse((child) => {
      if ((child as any).isMesh) {
        // Exclude collision proxies from visual mesh list
        if (child.userData.isCollisionProxy) return;

        const mesh = child as THREE.Mesh;
        this.allMeshes.push(mesh);
        this.originalMaterials.set(mesh, mesh.material);

        if (mesh.material instanceof THREE.MeshStandardMaterial || mesh.material instanceof THREE.MeshPhongMaterial) {
          mesh.material.side = THREE.DoubleSide;
        }
      }
    });

    // Categorize fingertip contact pads vs body meshes
    const leftTip = this.robot.links[this.spec.leftTipLink];
    const rightTip = this.robot.links[this.spec.rightTipLink];
    if (leftTip) {
      leftTip.traverse((c) => {
        if ((c as any).isMesh && !c.userData.isCollisionProxy) {
          this.leftFingerTipMeshes.push(c as THREE.Mesh);
          this.fingerTipMeshes.push(c as THREE.Mesh);
        }
      });
    }
    if (rightTip) {
      rightTip.traverse((c) => {
        if ((c as any).isMesh && !c.userData.isCollisionProxy) {
          this.rightFingerTipMeshes.push(c as THREE.Mesh);
          this.fingerTipMeshes.push(c as THREE.Mesh);
        }
      });
    }

    this.allMeshes.forEach((m) => {
      if (!this.fingerTipMeshes.includes(m)) {
        this.bodyMeshes.push(m);
      }
    });
  }

  /**
   * Builds low-poly box proxies (12 triangles each) parented to URDF links.
   * Total gripper collision complexity: 9 links * 12 = 108 triangles.
   * BVH computed ONCE per proxy in local space with { indirect: true, targetLeafSize: 10 }.
   */
  public setupCollisionProxies() {
    this.bodyProxies = [];
    this.allProxies = [];
    this.leftTipProxy = null;
    this.rightTipProxy = null;

    const proxyMat = new THREE.MeshBasicMaterial({ wireframe: true, visible: false });

    const linkKeys = Object.keys(this.robot.links);
    for (const linkName of linkKeys) {
      const link = this.robot.links[linkName];
      if (!link) continue;

      // Find the visual mesh direct child or within link (excluding nested links)
      let visualMesh: THREE.Mesh | null = null;
      for (const child of link.children) {
        if ((child as any).isMesh && !child.userData.isCollisionProxy) {
          visualMesh = child as THREE.Mesh;
          break;
        }
      }

      if (!visualMesh) {
        // Fallback traverse immediate child meshes
        link.traverse((c) => {
          if (!visualMesh && (c as any).isMesh && !c.userData.isCollisionProxy) {
            visualMesh = c as THREE.Mesh;
          }
        });
      }

      if (!visualMesh || !visualMesh.geometry) continue;

      if (!visualMesh.geometry.boundingBox) {
        visualMesh.geometry.computeBoundingBox();
      }
      const override = this.spec.proxyBoxes?.[linkName];
      const b = override
        ? new THREE.Box3(new THREE.Vector3(...override.min), new THREE.Vector3(...override.max))
        : visualMesh.geometry.boundingBox!;
      const sx = Math.max(0.002, b.max.x - b.min.x);
      const sy = Math.max(0.002, b.max.y - b.min.y);
      const sz = Math.max(0.002, b.max.z - b.min.z);

      const cx = (b.min.x + b.max.x) / 2;
      const cy = (b.min.y + b.max.y) / 2;
      const cz = (b.min.z + b.max.z) / 2;

      const boxGeom = new THREE.BoxGeometry(sx, sy, sz);
      boxGeom.translate(cx, cy, cz);
      boxGeom.computeBoundingBox();

      // Build BVH ONCE in local space
      (boxGeom as any).boundsTree = new MeshBVH(boxGeom, { indirect: true, targetLeafSize: 10 });

      const proxyMesh = new THREE.Mesh(boxGeom, proxyMat);
      proxyMesh.name = `proxy_${linkName}`;
      proxyMesh.visible = false;
      proxyMesh.userData.isCollisionProxy = true;
      proxyMesh.userData.linkName = linkName;
      proxyMesh.userData.parentVisual = visualMesh;

      link.add(proxyMesh);
      this.allProxies.push(proxyMesh);

      if (linkName === this.spec.leftTipLink) {
        this.leftTipProxy = proxyMesh;
      } else if (linkName === this.spec.rightTipLink) {
        this.rightTipProxy = proxyMesh;
      } else {
        this.bodyProxies.push(proxyMesh);
      }
    }
  }

  public getBodyProxies(): THREE.Mesh[] {
    if (this.bodyProxies.length === 0) this.setupCollisionProxies();
    return this.bodyProxies;
  }

  public getLeftTipProxy(): THREE.Mesh | null {
    if (!this.leftTipProxy) this.setupCollisionProxies();
    return this.leftTipProxy;
  }

  public getRightTipProxy(): THREE.Mesh | null {
    if (!this.rightTipProxy) this.setupCollisionProxies();
    return this.rightTipProxy;
  }

  public getAllProxies(): THREE.Mesh[] {
    if (this.allProxies.length === 0) this.setupCollisionProxies();
    return this.allProxies;
  }

  /**
   * Fast broad-phase world-space bounding box of the entire gripper.
   * Reuses scratch box to avoid allocations.
   */
  public getWholeBoundingBox(out: THREE.Box3): THREE.Box3 {
    if (this.allProxies.length === 0) this.setupCollisionProxies();
    out.makeEmpty();
    for (let i = 0; i < this.allProxies.length; i++) {
      const p = this.allProxies[i];
      if (p.geometry && p.geometry.boundingBox) {
        this._scratchBox.copy(p.geometry.boundingBox).applyMatrix4(p.matrixWorld);
        out.union(this._scratchBox);
      }
    }
    return out;
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
    this.bodyGroup.position.set(0, 0, -this.tcpOffset);
    this.root.updateMatrixWorld(true);
  }

  /**
   * Adjusts finger opening width in millimeters [0 to spec.maxOpeningWidth] using URDF kinematic joints
   */
  public setOpeningWidth(widthMm: number) {
    this.openingWidth = Math.max(
      GRIPPER_CONFIG.minOpeningWidth,
      Math.min(this.spec.maxOpeningWidth, widthMm)
    );

    // Closing "angle" 0.0 = fully open, 0.8 = fully closed (2F-85 knuckle angle; prismatic grippers map
    // it linearly to the finger travel so ContactGrip can treat all grippers alike).
    const angle = (1.0 - (this.openingWidth / this.spec.maxOpeningWidth)) * 0.8;
    this.setFingerAngles(angle, angle);
  }

  public getOpeningWidth(): number {
    return this.openingWidth;
  }

  /**
   * Sets finger closing angles independently for non-symmetric contact stops (0 = open, 0.8 = closed)
   */
  public setFingerAngles(leftAngle: number, rightAngle: number) {
    const j = this.robot.joints;
    // keep the reported opening in sync (contact animations drive the fingers through here)
    this.openingWidth = Math.max(0, Math.min(1, 1 - (leftAngle + rightAngle) / 2 / 0.8)) * this.spec.maxOpeningWidth;
    if (this.spec.kinematics === 'prismatic') {
      const travel = this.spec.fingerTravel ?? 0;
      const q = (a: number) => Math.max(0, Math.min(1, 1 - a / 0.8)) * travel;
      if (this.spec.leftFingerJoint && j[this.spec.leftFingerJoint]) {
        this.robot.setJointValue(this.spec.leftFingerJoint, q(leftAngle));
      }
      if (this.spec.rightFingerJoint && j[this.spec.rightFingerJoint]) {
        this.robot.setJointValue(this.spec.rightFingerJoint, q(rightAngle));
      }
      this.root.updateMatrixWorld(true);
      return;
    }

    // Robotiq 2F-85 URDF: angle 0.0 rad = fully open (85 mm), 0.8 rad = fully closed (0 mm)
    if (j['robotiq_85_left_knuckle_joint']) {
      this.robot.setJointValue('robotiq_85_left_knuckle_joint', leftAngle);
    }
    if (j['robotiq_85_left_inner_knuckle_joint']) {
      this.robot.setJointValue('robotiq_85_left_inner_knuckle_joint', leftAngle);
    }
    if (j['robotiq_85_left_finger_tip_joint']) {
      this.robot.setJointValue('robotiq_85_left_finger_tip_joint', -leftAngle);
    }

    if (j['robotiq_85_right_knuckle_joint']) {
      this.robot.setJointValue('robotiq_85_right_knuckle_joint', -rightAngle);
    }
    if (j['robotiq_85_right_inner_knuckle_joint']) {
      this.robot.setJointValue('robotiq_85_right_inner_knuckle_joint', -rightAngle);
    }
    if (j['robotiq_85_right_finger_tip_joint']) {
      this.robot.setJointValue('robotiq_85_right_finger_tip_joint', rightAngle);
    }

    this.root.updateMatrixWorld(true);
  }

  public getClosingAxis(): THREE.Vector3 {
    return new THREE.Vector3(1, 0, 0);
  }

  public getApproachAxis(): THREE.Vector3 {
    return new THREE.Vector3(0, 0, 1);
  }

  public getFingerTipMeshesGrouped(): { left: THREE.Mesh[]; right: THREE.Mesh[] } {
    return {
      left: this.leftFingerTipMeshes,
      right: this.rightFingerTipMeshes
    };
  }

  /**
   * Toggles collision highlight on specific penetrating visual meshes.
   * Zero per-frame allocations: swaps pre-made material references.
   */
  public highlightMeshes(penetratingMeshes: THREE.Mesh[]) {
    const hasCol = penetratingMeshes.length > 0;
    this.isColliding = hasCol;

    for (let i = 0; i < this.allMeshes.length; i++) {
      const mesh = this.allMeshes[i];
      if (penetratingMeshes.includes(mesh)) {
        if (mesh.material !== this.collisionMaterial) {
          mesh.material = this.collisionMaterial;
        }
      } else {
        const orig = this.originalMaterials.get(mesh);
        if (orig && mesh.material !== orig) {
          mesh.material = orig;
        }
      }
    }
  }

  /**
   * Toggles collision highlight across all meshes
   */
  public setCollisionHighlight(colliding: boolean) {
    if (this.isColliding === colliding) return;
    this.isColliding = colliding;

    for (let i = 0; i < this.allMeshes.length; i++) {
      const mesh = this.allMeshes[i];
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
    for (const p of this.allProxies) {
      p.geometry.dispose();
      (p.material as THREE.Material).dispose();
    }
  }
}
