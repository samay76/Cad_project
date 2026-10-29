import * as THREE from 'three';

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

    // 1. Broad-phase check: overall bounding boxes
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

        // Quick sphere rejection
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
}
