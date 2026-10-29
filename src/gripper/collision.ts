import * as THREE from 'three';
import {
  MeshBVH,
  acceleratedRaycast,
  computeBoundsTree,
  disposeBoundsTree
} from 'three-mesh-bvh';
import type { GripperModel } from './GripperModel';

// Initialize three-mesh-bvh extensions on Three.js prototypes once
(THREE.BufferGeometry.prototype as any).computeBoundsTree = computeBoundsTree;
(THREE.BufferGeometry.prototype as any).disposeBoundsTree = disposeBoundsTree;
THREE.Mesh.prototype.raycast = acceleratedRaycast;

export interface DetailedCollisionResult {
  status: 'none' | 'contact' | 'collision';
  hasCollision: boolean; // True only for body / non-pad penetration
  isContact: boolean;    // True when pads touch outer surface with gap < tolerance
  message: string;
  leftPadContact: boolean;
  rightPadContact: boolean;
  leftPadDist: number;
  rightPadDist: number;
  leftContactWorld?: THREE.Vector3;
  rightContactWorld?: THREE.Vector3;
  penetratingMeshes: THREE.Mesh[];
}

export class CollisionChecker {
  private static tempMatrix = new THREE.Matrix4();
  private static tempMatrixInv = new THREE.Matrix4();
  private static tempBox1 = new THREE.Box3();
  private static tempBox2 = new THREE.Box3();
  private static tempSphere1 = new THREE.Sphere();
  private static tempSphere2 = new THREE.Sphere();

  /**
   * Ensures all meshes in the object hierarchy have computed BVH bounds trees.
   * Only computes once per geometry unless force = true.
   */
  public static ensureBVH(root: THREE.Object3D | null, force = false): void {
    if (!root) return;

    root.traverse((child) => {
      if (child instanceof THREE.Mesh && child.geometry) {
        const geom = child.geometry as THREE.BufferGeometry & { boundsTree?: MeshBVH };
        if (!geom.boundsTree || force) {
          if (geom.boundsTree) {
            geom.disposeBoundsTree();
          }
          geom.computeBoundsTree();
        }
      }
    });
  }

  /**
   * Disposes BVH bounds trees on all child geometries
   */
  public static disposeBVH(root: THREE.Object3D | null): void {
    if (!root) return;
    root.traverse((child) => {
      if (child instanceof THREE.Mesh && child.geometry) {
        const geom = child.geometry as THREE.BufferGeometry & { boundsTree?: MeshBVH };
        if (geom.boundsTree) {
          geom.disposeBoundsTree();
        }
      }
    });
  }

  /**
   * Collects all CAD part meshes under the given root group
   */
  public static getCadMeshes(cadPartsRoot: THREE.Group | null): THREE.Mesh[] {
    if (!cadPartsRoot) return [];
    const meshes: THREE.Mesh[] = [];
    cadPartsRoot.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        meshes.push(child);
      }
    });
    return meshes;
  }

  /**
   * Checks full contact & collision state between gripper and CAD part using BVH.
   * - Green "none": no contact, no collision.
   * - Blue "contact": finger pads touch the CAD surface (gap < tolerance).
   * - Red "collision": palm, body, or knuckles penetrate into the CAD part.
   */
  public static evaluateState(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null,
    tolerance = 0.1 // mm
  ): DetailedCollisionResult {
    if (!cadPartsRoot) {
      return {
        status: 'none',
        hasCollision: false,
        isContact: false,
        message: 'No CAD model loaded',
        leftPadContact: false,
        rightPadContact: false,
        leftPadDist: Infinity,
        rightPadDist: Infinity,
        penetratingMeshes: []
      };
    }

    this.ensureBVH(cadPartsRoot);
    const cadMeshes = this.getCadMeshes(cadPartsRoot);
    if (cadMeshes.length === 0) {
      return {
        status: 'none',
        hasCollision: false,
        isContact: false,
        message: 'No CAD meshes',
        leftPadContact: false,
        rightPadContact: false,
        leftPadDist: Infinity,
        rightPadDist: Infinity,
        penetratingMeshes: []
      };
    }

    const bodyMeshes = gripperModel.getBodyMeshes();
    const { left: leftTipMeshes, right: rightTipMeshes } = gripperModel.getFingerTipMeshesGrouped();

    // Ensure BVH on gripper meshes once
    bodyMeshes.forEach((m) => {
      if (m.geometry && !m.geometry.boundingBox) m.geometry.computeBoundingBox();
      this.ensureBVH(m);
    });
    leftTipMeshes.forEach((m) => {
      if (m.geometry && !m.geometry.boundingBox) m.geometry.computeBoundingBox();
      this.ensureBVH(m);
    });
    rightTipMeshes.forEach((m) => {
      if (m.geometry && !m.geometry.boundingBox) m.geometry.computeBoundingBox();
      this.ensureBVH(m);
    });

    // Precompute CAD mesh info once (world bounding box + inverse matrix)
    const cadMeshInfo = cadMeshes.map((cMesh) => {
      if (!cMesh.geometry.boundingBox) cMesh.geometry.computeBoundingBox();
      const worldBox = new THREE.Box3().copy(cMesh.geometry.boundingBox!).applyMatrix4(cMesh.matrixWorld);
      const invMat = new THREE.Matrix4().copy(cMesh.matrixWorld).invert();
      const geom = cMesh.geometry as THREE.BufferGeometry & { boundsTree?: MeshBVH };
      return { cMesh, worldBox, invMat, geom };
    });

    // 1. Check body penetration (palm, housing, knuckles, finger backs)
    const penetratingMeshes: THREE.Mesh[] = [];

    for (const bMesh of bodyMeshes) {
      if (!bMesh.geometry || !bMesh.geometry.boundingBox) continue;
      this.tempBox1.copy(bMesh.geometry.boundingBox).applyMatrix4(bMesh.matrixWorld);

      for (const info of cadMeshInfo) {
        if (!info.geom.boundsTree) continue;

        // Fast bounding box rejection in world space
        if (!this.tempBox1.intersectsBox(info.worldBox)) continue;

        // Matrix transform from bMesh space to cMesh space (which is mm)
        this.tempMatrix.multiplyMatrices(info.invMat, bMesh.matrixWorld);

        // Fast boolean intersection via BVH
        const isIntersecting = info.geom.boundsTree.intersectsGeometry(bMesh.geometry, this.tempMatrix);
        if (isIntersecting) {
          penetratingMeshes.push(bMesh);
          break;
        }
      }
    }

    const hasBodyCollision = penetratingMeshes.length > 0;

    // 2. Measure distance and contact for left and right finger pads in CAD mm space
    let minLeftDist = Infinity;
    let minRightDist = Infinity;
    let leftContactWorld: THREE.Vector3 | undefined;
    let rightContactWorld: THREE.Vector3 | undefined;

    const testPadGroup = (
      tipMeshes: THREE.Mesh[]
    ): { dist: number; contactPoint?: THREE.Vector3; isIntersecting: boolean } => {
      let minDist = Infinity;
      let contactPt: THREE.Vector3 | undefined;
      let isIntersecting = false;

      for (const tMesh of tipMeshes) {
        if (!tMesh.geometry || !tMesh.geometry.boundingBox) continue;
        this.tempBox1.copy(tMesh.geometry.boundingBox).applyMatrix4(tMesh.matrixWorld);
        // Expand bounding box by tolerance + 5mm for proximity check
        this.tempBox1.expandByScalar(tolerance + 5.0);

        for (const info of cadMeshInfo) {
          if (!info.geom.boundsTree) continue;

          if (!this.tempBox1.intersectsBox(info.worldBox)) continue;

          // Transform from tMesh space to cMesh space (unscaled mm)
          this.tempMatrix.multiplyMatrices(info.invMat, tMesh.matrixWorld);

          // Fast intersection test
          if (info.geom.boundsTree.intersectsGeometry(tMesh.geometry, this.tempMatrix)) {
            isIntersecting = true;
            minDist = 0;
          }

          // Use bounded closestPointToGeometry with maxThreshold to prune distant BVH nodes
          const maxThreshold = tolerance + 5.0;
          const res = info.geom.boundsTree.closestPointToGeometry(
            tMesh.geometry,
            this.tempMatrix,
            {} as any,
            {} as any,
            0,
            maxThreshold
          );
          if (res) {
            if (res.distance < minDist) {
              minDist = res.distance;
              contactPt = res.point.clone().applyMatrix4(info.cMesh.matrixWorld);
            }
            if (res.distance <= 0.05) {
              isIntersecting = true;
            }
          }
        }
      }

      return { dist: minDist, contactPoint: contactPt, isIntersecting };
    };

    const leftRes = testPadGroup(leftTipMeshes);
    minLeftDist = leftRes.dist;
    leftContactWorld = leftRes.contactPoint;

    const rightRes = testPadGroup(rightTipMeshes);
    minRightDist = rightRes.dist;
    rightContactWorld = rightRes.contactPoint;

    const leftPadContact = minLeftDist <= tolerance || leftRes.isIntersecting;
    const rightPadContact = minRightDist <= tolerance || rightRes.isIntersecting;
    const isContact = leftPadContact || rightPadContact;

    // Determine overall status
    let status: 'none' | 'contact' | 'collision' = 'none';
    let message = 'No collision';

    if (hasBodyCollision) {
      status = 'collision';
      message = 'Collision: gripper body penetrating CAD part';
    } else if (leftPadContact && rightPadContact) {
      status = 'contact';
      message = 'Contact: both finger pads touching surface';
    } else if (leftPadContact) {
      status = 'contact';
      message = 'Contact: left finger pad touching surface';
    } else if (rightPadContact) {
      status = 'contact';
      message = 'Contact: right finger pad touching surface';
    }

    return {
      status,
      hasCollision: hasBodyCollision,
      isContact,
      message,
      leftPadContact,
      rightPadContact,
      leftPadDist: minLeftDist,
      rightPadDist: minRightDist,
      leftContactWorld,
      rightContactWorld,
      penetratingMeshes
    };
  }

  /**
   * Fast check for compatibility with existing code
   */
  public static checkCollision(
    gripperMeshes: THREE.Mesh[],
    cadPartsRoot: THREE.Group | null
  ): { hasCollision: boolean } {
    if (!cadPartsRoot || gripperMeshes.length === 0) return { hasCollision: false };

    this.ensureBVH(cadPartsRoot);
    const cadMeshes = this.getCadMeshes(cadPartsRoot);
    if (cadMeshes.length === 0) return { hasCollision: false };

    for (const gMesh of gripperMeshes) {
      if (!gMesh.geometry) continue;
      gMesh.geometry.computeBoundingBox();
      this.tempBox1.copy(gMesh.geometry.boundingBox!).applyMatrix4(gMesh.matrixWorld);

      for (const cMesh of cadMeshes) {
        if (!cMesh.geometry) continue;
        const cGeom = cMesh.geometry as THREE.BufferGeometry & { boundsTree?: MeshBVH };
        if (!cGeom.boundsTree) continue;

        cMesh.geometry.computeBoundingBox();
        this.tempBox2.copy(cMesh.geometry.boundingBox!).applyMatrix4(cMesh.matrixWorld);
        if (!this.tempBox1.intersectsBox(this.tempBox2)) continue;

        this.tempMatrixInv.copy(cMesh.matrixWorld).invert();
        this.tempMatrix.multiplyMatrices(this.tempMatrixInv, gMesh.matrixWorld);

        if (cGeom.boundsTree.intersectsGeometry(gMesh.geometry, this.tempMatrix)) {
          return { hasCollision: true };
        }
      }
    }

    return { hasCollision: false };
  }

  /**
   * Backs off along approach vector if gripper body intersects CAD solid
   */
  public static resolveApproachStandoff(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null,
    maxBackoffMm = 40
  ): number {
    if (!cadPartsRoot) return 0;

    const approachVec = new THREE.Vector3(0, 0, 1)
      .applyQuaternion(gripperModel.root.quaternion)
      .normalize();

    let backedOff = 0;
    const stepMm = 2.0;

    while (this.evaluateState(gripperModel, cadPartsRoot).hasCollision && backedOff < maxBackoffMm) {
      gripperModel.root.position.addScaledVector(approachVec, -stepMm);
      gripperModel.root.updateMatrixWorld(true);
      backedOff += stepMm;
    }

    return backedOff;
  }
}
