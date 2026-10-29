import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { SelectionManager } from './selection';
import { loadCADFile, type LoadedCADResult } from './loaders';
import type { CADModelMetadata, SelectionState, PartInfo } from '../types';

export class Viewer {
  private container: HTMLElement;
  private canvas: HTMLCanvasElement;
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;

  // Coordinate helpers
  private gridHelper: THREE.GridHelper | null = null;
  private axesHelper: THREE.AxesHelper | null = null;

  // Model & Selection
  private modelRoot: THREE.Group | null = null;
  private selectionManager: SelectionManager;
  private currentMetadata: CADModelMetadata | null = null;
  private currentParts: PartInfo[] = [];

  // State
  private isWireframe = false;
  private animationFrameId: number | null = null;
  private resizeObserver: ResizeObserver | null = null;

  // Mini orientation gizmo
  private gizmoCanvas: HTMLCanvasElement;
  private gizmoRenderer: THREE.WebGLRenderer;
  private gizmoScene: THREE.Scene;
  private gizmoCamera: THREE.PerspectiveCamera;

  // Callbacks
  private onModelLoadedCallback: ((metadata: CADModelMetadata, parts: PartInfo[]) => void) | null = null;
  private onSelectionChangedCallback: ((state: SelectionState) => void) | null = null;
  private onErrorCallback: ((errorMsg: string) => void) | null = null;
  private onLoadingCallback: ((isLoading: boolean, message?: string) => void) | null = null;

  constructor(container: HTMLElement) {
    this.container = container;

    // Enforce robotics Z-up convention as default in Three.js
    THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

    // Main Canvas & Renderer
    this.canvas = document.createElement('canvas');
    this.canvas.className = 'cad-canvas';
    this.container.appendChild(this.canvas);

    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      antialias: true,
      alpha: false,
      powerPreference: 'high-performance'
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0xf5f7fa, 1.0);
    this.renderer.shadowMap.enabled = false;

    // Scene
    this.scene = new THREE.Scene();

    // Camera with Z-up
    const aspect = this.container.clientWidth / (this.container.clientHeight || 1);
    this.camera = new THREE.PerspectiveCamera(45, aspect, 0.1, 100000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(200, -250, 180);

    // OrbitControls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.screenSpacePanning = true; // smooth right-click panning
    this.controls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT: THREE.MOUSE.PAN
    };
    this.controls.target.set(0, 0, 0);
    this.controls.update();

    // Setup Lighting
    this.setupLighting();

    // Setup Helpers (Grid & Axes)
    this.setupHelpers(200);

    // Setup Orientation Gizmo in corner
    const { gizmoCanvas, gizmoRenderer, gizmoScene, gizmoCamera } = this.setupGizmo();
    this.gizmoCanvas = gizmoCanvas;
    this.gizmoRenderer = gizmoRenderer;
    this.gizmoScene = gizmoScene;
    this.gizmoCamera = gizmoCamera;

    // Setup Selection Manager
    this.selectionManager = new SelectionManager(this.camera, this.canvas, this.scene);
    this.selectionManager.setSelectionCallback((state) => {
      if (this.onSelectionChangedCallback) {
        this.onSelectionChangedCallback(state);
      }
    });

    // Handle Window/Container Resize
    this.setupResizeObserver();

    // Start Rendering Loop
    this.animate = this.animate.bind(this);
    this.animate();
  }

  private setupLighting() {
    // Ambient light for general soft illumination
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
    this.scene.add(ambientLight);

    // Key directional light
    const keyLight = new THREE.DirectionalLight(0xffffff, 0.85);
    keyLight.position.set(150, -200, 300);
    this.scene.add(keyLight);

    // Fill light from opposite side
    const fillLight = new THREE.DirectionalLight(0xe2e8f0, 0.5);
    fillLight.position.set(-150, 200, 150);
    this.scene.add(fillLight);

    // Bottom bounce light
    const bounceLight = new THREE.DirectionalLight(0xf1f5f9, 0.3);
    bounceLight.position.set(0, 0, -200);
    this.scene.add(bounceLight);

    // Camera headlamp light to ensure shadowed angles remain clear
    const headlamp = new THREE.DirectionalLight(0xffffff, 0.35);
    this.camera.add(headlamp);
    this.scene.add(this.camera);
  }

  private setupHelpers(gridSize: number) {
    if (this.gridHelper) {
      this.scene.remove(this.gridHelper);
      this.gridHelper.dispose();
    }
    if (this.axesHelper) {
      this.scene.remove(this.axesHelper);
      this.axesHelper.dispose();
    }

    // Ground grid: with Z-up, ground is the XY plane!
    // GridHelper defaults to XZ plane, so rotate 90 deg around X
    const divisions = 20;
    this.gridHelper = new THREE.GridHelper(gridSize, divisions, 0x94a3b8, 0xe2e8f0);
    this.gridHelper.rotation.x = Math.PI / 2;
    this.gridHelper.position.set(0, 0, 0);
    this.scene.add(this.gridHelper);

    // XYZ Axes Helper: X=Red, Y=Green, Z=Blue
    this.axesHelper = new THREE.AxesHelper(gridSize * 0.4);
    this.axesHelper.renderOrder = 2;
    this.scene.add(this.axesHelper);
  }

  private setupGizmo() {
    const gizmoCanvas = document.createElement('canvas');
    gizmoCanvas.className = 'cad-orientation-gizmo';
    gizmoCanvas.width = 96;
    gizmoCanvas.height = 96;
    this.container.appendChild(gizmoCanvas);

    const gizmoRenderer = new THREE.WebGLRenderer({
      canvas: gizmoCanvas,
      alpha: true,
      antialias: true
    });
    gizmoRenderer.setSize(96, 96);
    gizmoRenderer.setClearColor(0x000000, 0.0);

    const gizmoScene = new THREE.Scene();
    const gizmoCamera = new THREE.PerspectiveCamera(40, 1, 0.1, 100);
    gizmoCamera.up.set(0, 0, 1);

    // Ambient light in gizmo
    gizmoScene.add(new THREE.AmbientLight(0xffffff, 0.9));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(2, -2, 3);
    gizmoScene.add(dirLight);

    // Central hub
    const hub = new THREE.Mesh(
      new THREE.SphereGeometry(0.18, 16, 16),
      new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.3 })
    );
    gizmoScene.add(hub);

    // Axes: X (Red), Y (Green), Z (Blue)
    const axisLength = 1.0;
    const makeAxis = (dir: THREE.Vector3, color: number, label: string) => {
      const group = new THREE.Group();

      // Axis shaft
      const cylinderGeom = new THREE.CylinderGeometry(0.04, 0.04, axisLength, 12);
      cylinderGeom.translate(0, axisLength / 2, 0);
      const shaftMat = new THREE.MeshStandardMaterial({ color, roughness: 0.2 });
      const shaft = new THREE.Mesh(cylinderGeom, shaftMat);

      // Arrow cone
      const coneGeom = new THREE.ConeGeometry(0.12, 0.28, 16);
      coneGeom.translate(0, axisLength + 0.14, 0);
      const cone = new THREE.Mesh(coneGeom, shaftMat);

      // Text label sprite
      const sprite = this.createAxisLabelSprite(label, color);
      sprite.position.set(0, axisLength + 0.45, 0);

      group.add(shaft);
      group.add(cone);
      group.add(sprite);

      // Rotate group towards target direction
      const defaultDir = new THREE.Vector3(0, 1, 0);
      const quat = new THREE.Quaternion().setFromUnitVectors(defaultDir, dir);
      group.setRotationFromQuaternion(quat);

      return group;
    };

    gizmoScene.add(makeAxis(new THREE.Vector3(1, 0, 0), 0xef4444, 'X'));
    gizmoScene.add(makeAxis(new THREE.Vector3(0, 1, 0), 0x10b981, 'Y'));
    gizmoScene.add(makeAxis(new THREE.Vector3(0, 0, 1), 0x2563eb, 'Z'));

    return { gizmoCanvas, gizmoRenderer, gizmoScene, gizmoCamera };
  }

  private createAxisLabelSprite(text: string, colorHex: number): THREE.Sprite {
    const canvas = document.createElement('canvas');
    canvas.width = 64;
    canvas.height = 64;
    const ctx = canvas.getContext('2d')!;

    ctx.fillStyle = '#' + colorHex.toString(16).padStart(6, '0');
    ctx.font = 'bold 44px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 32, 32);

    const texture = new THREE.CanvasTexture(canvas);
    const spriteMat = new THREE.SpriteMaterial({ map: texture, depthTest: false });
    const sprite = new THREE.Sprite(spriteMat);
    sprite.scale.set(0.45, 0.45, 1);
    return sprite;
  }

  private setupResizeObserver() {
    this.resizeObserver = new ResizeObserver(() => {
      this.handleResize();
    });
    this.resizeObserver.observe(this.container);
    this.handleResize();
  }

  public handleResize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;

    if (width === 0 || height === 0) return;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();

    this.renderer.setSize(width, height);
  }

  private animate() {
    this.animationFrameId = requestAnimationFrame(this.animate);

    this.controls.update();
    this.renderer.render(this.scene, this.camera);

    // Sync Orientation Gizmo
    if (this.gizmoCamera && this.gizmoRenderer && this.gizmoScene) {
      const dir = this.camera.position.clone().sub(this.controls.target).normalize();
      this.gizmoCamera.position.copy(dir.multiplyScalar(3.2));
      this.gizmoCamera.up.copy(this.camera.up);
      this.gizmoCamera.lookAt(0, 0, 0);
      this.gizmoRenderer.render(this.gizmoScene, this.gizmoCamera);
    }
  }

  /**
   * Loads a CAD file from ArrayBuffer
   */
  public async loadFile(buffer: ArrayBuffer, fileName: string): Promise<void> {
    try {
      this.notifyLoading(true, `Triangulating and loading ${fileName}...`);

      const result: LoadedCADResult = await loadCADFile(buffer, fileName);

      // Remove existing model
      if (this.modelRoot) {
        this.scene.remove(this.modelRoot);
        this.disposeObject(this.modelRoot);
        this.modelRoot = null;
      }

      this.modelRoot = result.modelRoot;
      this.scene.add(this.modelRoot);

      this.currentMetadata = result.metadata;
      this.currentParts = result.parts;

      // Adjust helpers based on model bounding box
      const box = result.metadata.boundingBox;
      const maxDim = Math.max(box.size.x, box.size.y, box.size.z, 20);
      const helperSize = Math.max(maxDim * 1.8, 50);

      this.setupHelpers(helperSize);
      if (this.gridHelper) {
        // Position ground grid at the bottom of the model
        this.gridHelper.position.set(0, 0, box.min.z);
      }

      // Initialize selection manager with new model
      this.selectionManager.setModel(
        this.modelRoot,
        result.selectableMeshes,
        new THREE.Vector3(box.size.x, box.size.y, box.size.z)
      );

      // Auto-center and fit to model
      this.fitToModel(false);

      this.notifyLoading(false);

      if (this.onModelLoadedCallback) {
        this.onModelLoadedCallback(result.metadata, result.parts);
      }
    } catch (err: any) {
      this.notifyLoading(false);
      const errMsg = err?.message || 'Failed to load CAD file.';
      if (this.onErrorCallback) {
        this.onErrorCallback(errMsg);
      }
    }
  }

  /**
   * Fits the camera view to frame the model
   */
  public fitToModel(animate = true) {
    if (!this.modelRoot || !this.currentMetadata) return;

    const box = this.currentMetadata.boundingBox;
    const center = new THREE.Vector3(box.center.x, box.center.y, box.center.z);
    const maxDim = Math.max(box.size.x, box.size.y, box.size.z);

    const fov = this.camera.fov * (Math.PI / 180);
    let cameraDistance = (maxDim / 2) / Math.tan(fov / 2);
    cameraDistance *= 1.6; // margin
    cameraDistance = Math.max(cameraDistance, 10);

    // Direction looking from front-right-above in Z-up system
    const dir = new THREE.Vector3(1.2, -1.5, 1.0).normalize();
    const newPos = center.clone().addScaledVector(dir, cameraDistance);

    this.moveCameraTo(newPos, center, animate);
  }

  /**
   * Resets the camera view to default orientation
   */
  public resetView() {
    this.fitToModel(true);
  }

  /**
   * Snaps camera to standard orthographic-like presets (Z-up robotics convention)
   */
  public setCameraView(preset: 'iso' | 'top' | 'front' | 'right') {
    if (!this.modelRoot || !this.currentMetadata) return;

    const box = this.currentMetadata.boundingBox;
    const center = new THREE.Vector3(box.center.x, box.center.y, box.center.z);
    const maxDim = Math.max(box.size.x, box.size.y, box.size.z);

    const fov = this.camera.fov * (Math.PI / 180);
    const cameraDistance = Math.max(((maxDim / 2) / Math.tan(fov / 2)) * 1.6, 10);

    let dir = new THREE.Vector3();
    if (preset === 'top') {
      dir.set(0, 0.0001, 1); // slight offset to prevent singularity with camera.up (0,0,1)
    } else if (preset === 'front') {
      dir.set(0, -1, 0);
    } else if (preset === 'right') {
      dir.set(1, 0, 0);
    } else {
      dir.set(1.2, -1.5, 1.0);
    }
    dir.normalize();

    const newPos = center.clone().addScaledVector(dir, cameraDistance);
    this.moveCameraTo(newPos, center, true);
  }

  /**
   * Toggles wireframe view on all CAD part meshes
   */
  public toggleWireframe(): boolean {
    this.isWireframe = !this.isWireframe;

    if (this.modelRoot) {
      this.modelRoot.traverse((child) => {
        if (child instanceof THREE.Mesh && child.userData.isCadPart) {
          if (child.material instanceof THREE.MeshStandardMaterial) {
            child.material.wireframe = this.isWireframe;
          }
        }
      });
    }

    return this.isWireframe;
  }

  public getIsWireframe(): boolean {
    return this.isWireframe;
  }

  /**
   * Programmatic part selection (e.g. from Parts List UI)
   */
  public selectPartByIndex(index: number) {
    this.selectionManager.selectPartByIndex(index);
  }

  public clearSelection() {
    this.selectionManager.clearSelection();
  }

  public getCamera(): THREE.PerspectiveCamera {
    return this.camera;
  }

  public getDomElement(): HTMLElement {
    return this.renderer.domElement;
  }

  public getScene(): THREE.Scene {
    return this.scene;
  }

  public getOrbitControls(): OrbitControls {
    return this.controls;
  }

  public getModelRoot(): THREE.Group | null {
    return this.modelRoot;
  }

  private moveCameraTo(position: THREE.Vector3, target: THREE.Vector3, animate = true) {
    if (!animate) {
      this.camera.position.copy(position);
      this.controls.target.copy(target);
      this.controls.update();
      return;
    }

    const startPos = this.camera.position.clone();
    const startTarget = this.controls.target.clone();
    const startTime = performance.now();
    const duration = 400; // ms

    const step = (now: number) => {
      const elapsed = now - startTime;
      const progress = Math.min(elapsed / duration, 1);
      // Ease out cubic
      const t = 1 - Math.pow(1 - progress, 3);

      this.camera.position.lerpVectors(startPos, position, t);
      this.controls.target.lerpVectors(startTarget, target, t);
      this.controls.update();

      if (progress < 1) {
        requestAnimationFrame(step);
      }
    };

    requestAnimationFrame(step);
  }

  private notifyLoading(isLoading: boolean, message?: string) {
    if (this.onLoadingCallback) {
      this.onLoadingCallback(isLoading, message);
    }
  }

  public setOnModelLoaded(cb: (metadata: CADModelMetadata, parts: PartInfo[]) => void) {
    this.onModelLoadedCallback = cb;
  }

  public setOnSelectionChanged(cb: (state: SelectionState) => void) {
    this.onSelectionChangedCallback = cb;
  }

  public setOnError(cb: (msg: string) => void) {
    this.onErrorCallback = cb;
  }

  public setOnLoading(cb: (isLoading: boolean, msg?: string) => void) {
    this.onLoadingCallback = cb;
  }

  private disposeObject(obj: THREE.Object3D) {
    obj.traverse((child) => {
      if (child instanceof THREE.Mesh) {
        child.geometry.dispose();
        if (Array.isArray(child.material)) {
          child.material.forEach((m) => m.dispose());
        } else {
          child.material.dispose();
        }
      }
    });
  }

  public dispose() {
    if (this.animationFrameId !== null) {
      cancelAnimationFrame(this.animationFrameId);
    }
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
    }
    this.selectionManager.dispose();
    this.renderer.dispose();
    this.gizmoRenderer.dispose();
  }
}
