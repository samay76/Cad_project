import * as THREE from 'three';
import type { ClickedPointInfo, SelectionState } from '../types';

export type SelectionCallback = (state: SelectionState) => void;

export class SelectionManager {
  private camera: THREE.Camera;
  private domElement: HTMLElement;
  private modelRoot: THREE.Group | null = null;
  private selectableMeshes: THREE.Mesh[] = [];

  private raycaster = new THREE.Raycaster();
  private mouse = new THREE.Vector2();

  private hoveredMesh: THREE.Mesh | null = null;
  private selectedMesh: THREE.Mesh | null = null;
  private clickedPointInfo: ClickedPointInfo | null = null;

  // Visual marker in 3D for clicked point & normal
  private pickMarkerGroup: THREE.Group = new THREE.Group();
  private markerSphere: THREE.Mesh;
  private markerArrow: THREE.ArrowHelper;

  private pointerDownPos = { x: 0, y: 0 };
  private onSelectionChange: SelectionCallback | null = null;
  private onRequestRender: (() => void) | null = null;
  private isDragging = false;

  private boundOnPointerMove: (e: PointerEvent) => void;
  private boundOnPointerDown: (e: PointerEvent) => void;
  private boundOnPointerUp: (e: PointerEvent) => void;
  private boundOnPointerLeave: () => void;

  constructor(camera: THREE.Camera, domElement: HTMLElement, scene: THREE.Scene) {
    this.camera = camera;
    this.domElement = domElement;

    // BVH-accelerated raycasting with firstHitOnly = true
    (this.raycaster as any).firstHitOnly = true;

    // Build pick marker (sphere + normal vector arrow)
    const sphereGeom = new THREE.SphereGeometry(1.5, 16, 16);
    const sphereMat = new THREE.MeshBasicMaterial({ color: 0x2563eb });
    this.markerSphere = new THREE.Mesh(sphereGeom, sphereMat);

    const defaultDir = new THREE.Vector3(0, 0, 1);
    const defaultOrigin = new THREE.Vector3(0, 0, 0);
    this.markerArrow = new THREE.ArrowHelper(defaultDir, defaultOrigin, 15, 0x1d4ed8, 4, 2);

    this.pickMarkerGroup.add(this.markerSphere);
    this.pickMarkerGroup.add(this.markerArrow);
    this.pickMarkerGroup.visible = false;
    scene.add(this.pickMarkerGroup);

    this.boundOnPointerMove = this.onPointerMove.bind(this);
    this.boundOnPointerDown = this.onPointerDown.bind(this);
    this.boundOnPointerUp = this.onPointerUp.bind(this);
    this.boundOnPointerLeave = this.onPointerLeave.bind(this);

    this.attachEvents();
  }

  private attachEvents() {
    this.domElement.addEventListener('pointermove', this.boundOnPointerMove);
    this.domElement.addEventListener('pointerdown', this.boundOnPointerDown);
    this.domElement.addEventListener('pointerup', this.boundOnPointerUp);
    this.domElement.addEventListener('pointerleave', this.boundOnPointerLeave);
  }

  public setModel(modelRoot: THREE.Group, selectableMeshes: THREE.Mesh[], boundingBoxSize?: THREE.Vector3) {
    this.clearSelection();
    this.modelRoot = modelRoot;
    this.selectableMeshes = selectableMeshes;

    // Adjust marker scale proportionally to model scale
    if (boundingBoxSize) {
      const maxDim = Math.max(boundingBoxSize.x, boundingBoxSize.y, boundingBoxSize.z);
      const scale = Math.max(0.02 * maxDim, 0.5);
      this.markerSphere.scale.set(scale, scale, scale);
      this.markerArrow.setLength(scale * 8, scale * 2.5, scale * 1.5);
    }
  }

  public setSelectionCallback(callback: SelectionCallback) {
    this.onSelectionChange = callback;
  }

  public setOnRequestRender(callback: () => void) {
    this.onRequestRender = callback;
  }

  /**
   * Disables hover raycasting while TransformControls is dragging or camera is orbiting
   */
  public setIsDragging(dragging: boolean) {
    this.isDragging = dragging;
    if (dragging && this.hoveredMesh) {
      this.clearHover();
      this.domElement.style.cursor = 'default';
      if (this.onRequestRender) this.onRequestRender();
    }
  }

  private updateNormalizedMouse(e: PointerEvent) {
    const rect = this.domElement.getBoundingClientRect();
    this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  }

  private onPointerMove(e: PointerEvent) {
    // Disable hover raycasting while dragging gizmo or orbiting camera
    if (this.isDragging || this.selectableMeshes.length === 0) return;

    this.updateNormalizedMouse(e);
    this.raycaster.setFromCamera(this.mouse, this.camera);

    const intersects = this.raycaster.intersectObjects(this.selectableMeshes, false);

    if (intersects.length > 0) {
      const topMesh = intersects[0].object as THREE.Mesh;
      if (this.hoveredMesh !== topMesh) {
        this.clearHover();
        this.hoveredMesh = topMesh;
        if (this.hoveredMesh !== this.selectedMesh) {
          this.applyHoverHighlight(this.hoveredMesh, true);
        }
        if (this.onRequestRender) this.onRequestRender();
      }
      this.domElement.style.cursor = 'pointer';
    } else {
      if (this.hoveredMesh) {
        this.clearHover();
        if (this.onRequestRender) this.onRequestRender();
      }
      this.domElement.style.cursor = 'default';
    }
  }

  private onPointerLeave() {
    this.clearHover();
    this.domElement.style.cursor = 'default';
  }

  private onPointerDown(e: PointerEvent) {
    this.pointerDownPos = { x: e.clientX, y: e.clientY };
  }

  private onPointerUp(e: PointerEvent) {
    const deltaX = Math.abs(e.clientX - this.pointerDownPos.x);
    const deltaY = Math.abs(e.clientY - this.pointerDownPos.y);

    // If moved more than 4 pixels, user was dragging/orbiting the camera
    if (deltaX > 4 || deltaY > 4) {
      return;
    }

    if (this.selectableMeshes.length === 0) return;

    this.updateNormalizedMouse(e);
    this.raycaster.setFromCamera(this.mouse, this.camera);

    const intersects = this.raycaster.intersectObjects(this.selectableMeshes, false);

    if (intersects.length > 0) {
      const intersect = intersects[0];
      const clickedMesh = intersect.object as THREE.Mesh;
      this.selectMeshWithIntersection(clickedMesh, intersect);
    } else {
      // Clicked on empty space: deselect
      this.clearSelection();
      this.notifySelection();
    }
  }

  /**
   * Applies selection to a mesh along with click coordinate calculation
   */
  private selectMeshWithIntersection(mesh: THREE.Mesh, intersect: THREE.Intersection) {
    if (this.selectedMesh && this.selectedMesh !== mesh) {
      this.applySelectHighlight(this.selectedMesh, false);
    }

    this.selectedMesh = mesh;
    this.applySelectHighlight(this.selectedMesh, true);

    // Calculate surface point in model coordinate frame (mm)
    let modelPoint = intersect.point.clone();
    if (this.modelRoot) {
      modelPoint = this.modelRoot.worldToLocal(modelPoint.clone());
    }

    // Calculate surface normal in model coordinate frame
    let worldNormal = new THREE.Vector3(0, 0, 1);
    if (intersect.face) {
      const normalMatrix = new THREE.Matrix3().getNormalMatrix(mesh.matrixWorld);
      worldNormal = intersect.face.normal.clone().applyMatrix3(normalMatrix).normalize();
    }

    let modelNormal = worldNormal.clone();
    if (this.modelRoot) {
      const rootInv = new THREE.Matrix4().copy(this.modelRoot.matrixWorld).invert();
      modelNormal.transformDirection(rootInv).normalize();
    }

    const userData = mesh.userData;
    this.clickedPointInfo = {
      point: {
        x: Number(modelPoint.x.toFixed(3)),
        y: Number(modelPoint.y.toFixed(3)),
        z: Number(modelPoint.z.toFixed(3))
      },
      normal: {
        x: Number(modelNormal.x.toFixed(3)),
        y: Number(modelNormal.y.toFixed(3)),
        z: Number(modelNormal.z.toFixed(3))
      },
      worldPoint: {
        x: Number(intersect.point.x.toFixed(3)),
        y: Number(intersect.point.y.toFixed(3)),
        z: Number(intersect.point.z.toFixed(3))
      },
      worldNormal: {
        x: Number(worldNormal.x.toFixed(3)),
        y: Number(worldNormal.y.toFixed(3)),
        z: Number(worldNormal.z.toFixed(3))
      },
      partIndex: userData.partIndex ?? 0,
      partName: userData.partName ?? mesh.name ?? 'Part'
    };

    // Position pick marker in 3D scene
    this.markerSphere.position.copy(intersect.point);
    this.markerArrow.position.copy(intersect.point);
    this.markerArrow.setDirection(worldNormal);
    this.pickMarkerGroup.visible = true;

    this.notifySelection();
  }

  /**
   * Programmatic selection by part ID or index (e.g. from Parts List click)
   */
  public selectPartByIndex(index: number) {
    const mesh = this.selectableMeshes.find((m) => m.userData.partIndex === index);
    if (!mesh) return;

    if (this.selectedMesh && this.selectedMesh !== mesh) {
      this.applySelectHighlight(this.selectedMesh, false);
    }

    this.selectedMesh = mesh;
    this.applySelectHighlight(this.selectedMesh, true);

    // Compute bounding center of this part to give sensible default coordinate if not clicked on face
    const partBox = new THREE.Box3().setFromObject(mesh);
    const center = partBox.getCenter(new THREE.Vector3());
    let modelCenter = center.clone();
    if (this.modelRoot) {
      modelCenter = this.modelRoot.worldToLocal(modelCenter.clone());
    }

    this.clickedPointInfo = {
      point: {
        x: Number(modelCenter.x.toFixed(3)),
        y: Number(modelCenter.y.toFixed(3)),
        z: Number(modelCenter.z.toFixed(3))
      },
      normal: { x: 0, y: 0, z: 1 },
      worldPoint: {
        x: Number(center.x.toFixed(3)),
        y: Number(center.y.toFixed(3)),
        z: Number(center.z.toFixed(3))
      },
      worldNormal: { x: 0, y: 0, z: 1 },
      partIndex: mesh.userData.partIndex ?? index,
      partName: mesh.userData.partName ?? mesh.name
    };

    this.markerSphere.position.copy(center);
    this.markerArrow.position.copy(center);
    this.markerArrow.setDirection(new THREE.Vector3(0, 0, 1));
    this.pickMarkerGroup.visible = true;

    this.notifySelection();
  }

  private applyHoverHighlight(mesh: THREE.Mesh, isHovered: boolean) {
    const mat = mesh.material;
    if (mat instanceof THREE.MeshStandardMaterial) {
      if (isHovered) {
        mat.emissive.setHex(0x60a5fa);
        mat.emissiveIntensity = 0.35;
      } else {
        mat.emissive.setHex(0x000000);
        mat.emissiveIntensity = 0;
      }
    }
  }

  private applySelectHighlight(mesh: THREE.Mesh, isSelected: boolean) {
    const mat = mesh.material;
    if (mat instanceof THREE.MeshStandardMaterial) {
      if (isSelected) {
        mat.emissive.setHex(0x2563eb);
        mat.emissiveIntensity = 0.65;
      } else {
        mat.emissive.setHex(0x000000);
        mat.emissiveIntensity = 0;
      }
    }

    if (mesh.userData.edges instanceof THREE.LineSegments) {
      const edgeMat = mesh.userData.edges.material;
      if (edgeMat instanceof THREE.LineBasicMaterial) {
        edgeMat.color.setHex(isSelected ? 0x1d4ed8 : 0x334155);
        edgeMat.opacity = isSelected ? 1.0 : 0.6;
      }
    }
  }

  private clearHover() {
    if (this.hoveredMesh) {
      if (this.hoveredMesh !== this.selectedMesh) {
        this.applyHoverHighlight(this.hoveredMesh, false);
      }
      this.hoveredMesh = null;
    }
  }

  public clearSelection() {
    this.clearHover();
    if (this.selectedMesh) {
      this.applySelectHighlight(this.selectedMesh, false);
      this.selectedMesh = null;
    }
    this.clickedPointInfo = null;
    this.pickMarkerGroup.visible = false;
    if (this.onRequestRender) this.onRequestRender();
  }

  private notifySelection() {
    if (this.onRequestRender) this.onRequestRender();
    if (this.onSelectionChange) {
      this.onSelectionChange({
        selectedPartId: this.selectedMesh?.userData.partId ?? null,
        selectedPartIndex: this.selectedMesh?.userData.partIndex ?? null,
        selectedPartName: this.selectedMesh?.userData.partName ?? null,
        clickedPoint: this.clickedPointInfo
      });
    }
  }

  public getSelectionState(): SelectionState {
    return {
      selectedPartId: this.selectedMesh?.userData.partId ?? null,
      selectedPartIndex: this.selectedMesh?.userData.partIndex ?? null,
      selectedPartName: this.selectedMesh?.userData.partName ?? null,
      clickedPoint: this.clickedPointInfo
    };
  }

  public dispose() {
    this.domElement.removeEventListener('pointermove', this.boundOnPointerMove);
    this.domElement.removeEventListener('pointerdown', this.boundOnPointerDown);
    this.domElement.removeEventListener('pointerup', this.boundOnPointerUp);
    this.domElement.removeEventListener('pointerleave', this.boundOnPointerLeave);
  }
}
