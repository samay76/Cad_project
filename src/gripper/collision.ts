import * as THREE from 'three';
import type { GripperModel } from './GripperModel';

export interface CollisionResult {
  hasCollision: boolean;
  collidingPartIndex?: number;
  collidingPartName?: string;
}

export class CollisionChecker {
  private static tempBox1 = new THREE.Box3();
  private static tempBox2 = new THREE.Box3();
  private static tempSphere1 = new THREE.Sphere();
  private static tempSphere2 = new THREE.Sphere();

  /**
   * Fast, reliable interference check between gripper meshes and CAD part meshes
   */
  public static checkCollision(
    gripperMeshes: THREE.Mesh[],
    cadPartsRoot: THREE.Group | null
  ): CollisionResult {
    if (!cadPartsRoot || gripperMeshes.length === 0) {
      return { hasCollision: false };
    }

    // 1. Broad-phase check: overall bounding box of CAD model
    this.tempBox1.setFromObject(cadPartsRoot);
    if (this.tempBox1.isEmpty()) {
      return { hasCollision: false };
    }

    // Collect CAD meshes
    const cadMeshes: THREE.Mesh[] = [];
    cadPartsRoot.traverse((child) => {
      if (child instanceof THREE.Mesh && child.userData.isCadPart) {
        cadMeshes.push(child);
      }
    });

    if (cadMeshes.length === 0) {
      return { hasCollision: false };
    }

    // 2. Check each gripper mesh against candidate CAD meshes
    for (const gMesh of gripperMeshes) {
      this.tempBox1.setFromObject(gMesh);
      if (this.tempBox1.isEmpty()) continue;

      this.tempBox1.getBoundingSphere(this.tempSphere1);

      for (const cMesh of cadMeshes) {
        this.tempBox2.setFromObject(cMesh);
        if (this.tempBox2.isEmpty()) continue;

        // Sphere rejection
        this.tempBox2.getBoundingSphere(this.tempSphere2);
        const dist = this.tempSphere1.center.distanceTo(this.tempSphere2.center);
        if (dist > this.tempSphere1.radius + this.tempSphere2.radius) {
          continue;
        }

        // Bounding box intersection check
        if (this.tempBox1.intersectsBox(this.tempBox2)) {
          return {
            hasCollision: true,
            collidingPartIndex: cMesh.userData.partIndex,
            collidingPartName: cMesh.userData.partName || cMesh.name
          };
        }
      }
    }

    return { hasCollision: false };
  }

  /**
   * Checks collision specifically for the fingertip contact pads against the CAD model
   */
  public static checkFingertipCollision(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null
  ): boolean {
    const tipMeshes = gripperModel.getFingerTipMeshes();
    const res = this.checkCollision(tipMeshes, cadPartsRoot);
    return res.hasCollision;
  }

  /**
   * Checks collision specifically for the gripper palm/housing/body against the CAD model
   */
  public static checkBodyCollision(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null
  ): boolean {
    const bodyMeshes = gripperModel.getBodyMeshes();
    const res = this.checkCollision(bodyMeshes, cadPartsRoot);
    return res.hasCollision;
  }

  /**
   * Finds the exact contact opening width where finger pads contact the CAD part.
   * If a CAD part is between the fingers, returns contact width in mm (clamped in [0, 85]).
   * If no part is between the fingers, returns 0.
   */
  public static findGraspContactWidth(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null
  ): number {
    if (!cadPartsRoot) return 0;

    const origWidth = gripperModel.getOpeningWidth();

    // Check if fully closed collides with CAD part
    gripperModel.setOpeningWidth(0.5);
    gripperModel.root.updateMatrixWorld(true);
    const closedCollides = this.checkFingertipCollision(gripperModel, cadPartsRoot);

    if (!closedCollides) {
      // No part between fingers even when closed
      gripperModel.setOpeningWidth(origWidth);
      gripperModel.root.updateMatrixWorld(true);
      return 0;
    }

    // Binary search for contact boundary between [0, 85 mm]
    let low = 0.0;
    let high = 85.0;
    let contactWidth = 0.0;

    for (let i = 0; i < 9; i++) {
      const mid = (low + high) / 2;
      gripperModel.setOpeningWidth(mid);
      gripperModel.root.updateMatrixWorld(true);

      if (this.checkFingertipCollision(gripperModel, cadPartsRoot)) {
        // Still in collision at mid width -> contact surface is at or above mid
        contactWidth = mid;
        low = mid;
      } else {
        high = mid;
      }
    }

    // Restore original opening width
    gripperModel.setOpeningWidth(origWidth);
    gripperModel.root.updateMatrixWorld(true);

    return Number(contactWidth.toFixed(1));
  }

  /**
   * Ensures the gripper body/palm does NOT penetrate inside the CAD model.
   * If penetrating, backs off along the approach axis until the gripper sits on the surface.
   */
  public static resolveApproachStandoff(
    gripperModel: GripperModel,
    cadPartsRoot: THREE.Group | null,
    maxBackoffMm = 80
  ): number {
    if (!cadPartsRoot) return 0;

    // Approach vector in world coordinates (+Z of TCP)
    const approachVec = new THREE.Vector3(0, 0, 1)
      .applyQuaternion(gripperModel.root.quaternion)
      .normalize();

    let backedOff = 0;
    const stepMm = 1.0;

    while (this.checkBodyCollision(gripperModel, cadPartsRoot) && backedOff < maxBackoffMm) {
      // Step backwards along approach direction (out of the part)
      gripperModel.root.position.addScaledVector(approachVec, -stepMm);
      gripperModel.root.updateMatrixWorld(true);
      backedOff += stepMm;
    }

    return backedOff;
  }
}
