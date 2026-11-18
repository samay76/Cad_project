/**
 * Small 3D view of the CAD model built from the depth camera (inside CameraScanPanel).
 * Drag to rotate, scroll to zoom; it turns slowly on its own until touched.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

export class ScanModelPreview {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(35, 4 / 3, 1, 10000);
  private controls: OrbitControls;
  private model: THREE.Group | null = null;
  private raf: number | null = null;
  private resizeObserver: ResizeObserver;

  constructor(private container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.domElement.className = 'camera-scan-model-canvas';
    this.renderer.domElement.draggable = false;
    container.appendChild(this.renderer.domElement);

    this.scene.background = new THREE.Color(0x0f172a);
    this.camera.up.set(0, 0, 1); // z-up, same as the main viewer and the scan (z = 0 on the table)
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x334155, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.8);
    sun.position.set(1, -1.5, 2.5);
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight(0xffffff, 0.5);
    fill.position.set(-1.5, 1, 0.5);
    this.scene.add(fill);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.autoRotate = true;
    this.controls.autoRotateSpeed = 1.5;
    this.controls.addEventListener('start', () => (this.controls.autoRotate = false));

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
  }

  public hasModel() {
    return !!this.model;
  }

  /** Shows a binary/ASCII STL in mm (z = 0 on the table). Returns its size in mm. */
  public setModel(buffer: ArrayBuffer): THREE.Vector3 {
    if (this.model) {
      this.scene.remove(this.model);
      this.model.traverse((o) => {
        if (o instanceof THREE.Mesh || o instanceof THREE.LineSegments) {
          o.geometry.dispose();
          (o.material as THREE.Material).dispose();
        }
      });
    }
    const geometry = new STLLoader().parse(buffer);
    geometry.computeBoundingBox();
    const box = geometry.boundingBox!;
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());

    const group = new THREE.Group();
    group.add(
      new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({ color: 0xb8bcc6, roughness: 0.55, metalness: 0.1, flatShading: true })
      )
    );
    const span = Math.max(size.x, size.y, 10);
    const grid = new THREE.GridHelper(span * 1.6, 16, 0x475569, 0x1e293b);
    grid.rotation.x = Math.PI / 2; // XY plane = the table
    grid.position.set(center.x, center.y, box.min.z);
    group.add(grid);
    this.model = group;
    this.scene.add(group);

    const radius = size.length() / 2 || 50;
    const dist = radius / Math.sin(THREE.MathUtils.degToRad(this.camera.fov / 2)) * 1.05;
    this.controls.target.copy(center);
    this.camera.position.copy(center).add(new THREE.Vector3(0.9, -1.3, 1.0).normalize().multiplyScalar(dist));
    this.camera.near = dist / 100;
    this.camera.far = dist * 100;
    this.camera.updateProjectionMatrix();
    this.controls.autoRotate = true;
    this.controls.update();
    return size;
  }

  public start() {
    if (this.raf !== null) return;
    this.resize();
    const loop = () => {
      this.raf = requestAnimationFrame(loop);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };
    loop();
  }

  public stop() {
    if (this.raf !== null) cancelAnimationFrame(this.raf);
    this.raf = null;
  }

  private resize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }
}
