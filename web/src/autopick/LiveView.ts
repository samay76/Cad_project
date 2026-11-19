/**
 * Live view of a running Sim / Real Sim / Preview in the web app: the same cell MuJoCo simulates (table, fixture,
 * tray, EC66, 2F-85, part), drawn here and moved by the job's live stream (cell/web_view.py, server /api/mujoco/view).
 *
 * The cell is drawn in the frame of the 3D view (the part's CAD frame, mm): T_view maps the sim world into it. While
 * the live view is on, the teaching objects (part, fixture, gripper, plan drawing) are hidden; "Back to teaching"
 * shows them again. Frames are played ~150 ms behind the newest one, interpolated, so the motion is smooth.
 */
import * as THREE from 'three';
import { toCreasedNormals } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import type { Viewer } from '../viewer/Viewer';
import { buildRoom, materialFor } from './cellLook';

interface SceneGeom {
  type: 'plane' | 'sphere' | 'capsule' | 'ellipsoid' | 'cylinder' | 'box' | 'mesh';
  size: number[];
  rgba: number[];
  mesh?: string;
  body: number | null;
  bodyName?: string;
  geomName?: string;
  material?: string;
  pos: number[];
  quat: number[]; // w x y z
}

interface ViewScene {
  version: number;
  label: string;
  bodies: string[];
  frameFloats: number;
  T_view: number[][];
  floorZ?: number; // floor height in the sim world (m)
  look?: Record<string, string>; // config web_view *_color
  geoms: SceneGeom[];
}

const POLL_MS = 100;
const LAG_S = 0.15; // play this far behind the newest frame
const MAX_LAG_S = 0.6; // further behind than this: jump ahead
const meshCache = new Map<string, Promise<THREE.BufferGeometry>>(); // per page load: the table is fetched once

const loadMesh = (key: string) => {
  let p = meshCache.get(key);
  if (!p) {
    p = fetch(`/api/mujoco/view/mesh/${key}`)
      .then((r) => {
        if (!r.ok) throw new Error(`mesh ${key}: HTTP ${r.status}`);
        return r.arrayBuffer();
      })
      .then((buf) => {
        const head = new Uint32Array(buf, 0, 2);
        const nv = head[0];
        const nf = head[1];
        const verts = new Float32Array(buf, 8, nv * 3);
        const faces = new Uint32Array(buf, 8 + nv * 12, nf * 3);
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.BufferAttribute(verts, 3));
        g.setIndex(new THREE.BufferAttribute(faces, 1));
        return toCreasedNormals(g, Math.PI / 6); // CAD look: sharp edges, smooth curves
      });
    meshCache.set(key, p);
  }
  return p;
};

const primitive = (g: SceneGeom): THREE.BufferGeometry | null => {
  const [a, b, c] = g.size;
  switch (g.type) {
    case 'box':
      return new THREE.BoxGeometry(2 * a, 2 * b, 2 * c);
    case 'sphere':
      return new THREE.SphereGeometry(a, 24, 16);
    case 'ellipsoid':
      return new THREE.SphereGeometry(1, 24, 16).scale(a, b, c);
    case 'cylinder':
      return new THREE.CylinderGeometry(a, a, 2 * b, 32).rotateX(Math.PI / 2); // MuJoCo: along Z
    case 'capsule':
      return new THREE.CapsuleGeometry(a, 2 * b, 8, 16).rotateX(Math.PI / 2);
    default:
      return null; // planes: the viewer draws its own ground
  }
};

/** v = [x, y, z, qw, qx, qy, qz] (MuJoCo order) */
const setPose = (obj: THREE.Object3D, v: ArrayLike<number>) => {
  obj.position.set(v[0], v[1], v[2]);
  obj.quaternion.set(v[4], v[5], v[6], v[3]);
};

export class LiveView {
  private root = new THREE.Group(); // sim world (m) -> 3D view (mm)
  private bodies: THREE.Group[] = [];
  private scene: ViewScene | null = null;
  private runId: string | null = null;
  private frames: Float32Array[] = []; // newest last
  private nextFrame = 0;
  private playT: number | null = null;
  private lastWall = 0;
  private pollTimer: number | null = null;
  private raf: number | null = null;
  private ended = false;
  private overlay: HTMLDivElement;
  private session = 0;
  private framed = false; // camera fitted to the robot once per live view
  /** none: nothing; cell: the empty cell around the part while teaching (EC66 at home); live: a running job */
  private mode: 'none' | 'cell' | 'live' = 'none';

  constructor(
    private viewer: Viewer,
    private hideTeaching: (hide: boolean) => void
  ) {
    this.root.matrixAutoUpdate = false;
    this.root.name = 'live_cell';
    this.root.visible = false;
    viewer.getScene().add(this.root);
    this.overlay = document.createElement('div');
    this.overlay.className = 'live-view-overlay';
    this.overlay.innerHTML = `
      <div class="live-view-head"><span class="live-dot"></span><span class="live-title">Live</span>
        <button class="btn btn-secondary btn-sm live-close" title="Hide the live cell and show the part, fixture and gripper again">Back to teaching</button></div>
      <div class="live-view-msg"></div>`;
    this.overlay.querySelector('.live-close')!.addEventListener('click', () => this.close());
    (viewer.getDomElement().parentElement ?? document.body).appendChild(this.overlay);
  }

  public get active() {
    return this.runId !== null;
  }

  /** Teaching: the cell (table, tray, room, EC66 at home) around the part, so points are marked in the workspace.
   * The part stays where the web app shows it (seated on the fixture, as in the simulation). */
  public async showCell() {
    if (this.mode !== 'none') return;
    this.clearScene();
    this.mode = 'cell';
    const session = ++this.session;
    try {
      const r = await fetch('/api/cell/scene', { cache: 'no-store' });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      await this.build((await r.json()) as ViewScene, session, true);
    } catch (err) {
      console.warn('Cell view:', err);
      if (session === this.session) {
        this.mode = 'none';
        this.viewer.setLiveLook(false);
      }
    }
  }

  /** Shows the live cell of a job that was just started in run `runId`. */
  public start(runId: string, title: string) {
    if (this.runId !== null) this.viewer.stopAnimation();
    this.clearScene();
    this.mode = 'live';
    const session = ++this.session;
    this.runId = runId;
    this.ended = false;
    this.overlay.classList.add('visible', 'is-live');
    this.overlay.querySelector('.live-title')!.textContent = title;
    this.setMessage('Starting... (the first run of a part builds its scene)');
    this.hideTeaching(true);
    this.viewer.startAnimation();
    const waitForScene = async () => {
      if (session !== this.session) return;
      try {
        const r = await fetch(`/api/mujoco/view/scene?id=${encodeURIComponent(runId)}`, { cache: 'no-store' });
        if (r.ok) {
          await this.build((await r.json()) as ViewScene, session);
          return;
        }
      } catch {
        // server busy: try again
      }
      if (!this.ended) window.setTimeout(waitForScene, 400);
    };
    waitForScene();
  }

  /** The job ended: keep its last frame on screen with the result. */
  public finish(ok: boolean, message: string) {
    if (!this.active) return;
    this.ended = true;
    this.setMessage(message, ok ? 'ok' : 'error');
    this.overlay.classList.remove('is-live');
    if (this.scene) this.poll(); // the last frames
    else this.close(); // ended before the scene was built (e.g. a planning error): nothing to show
  }

  public setMessage(text: string, kind: 'info' | 'ok' | 'error' = 'info') {
    const el = this.overlay.querySelector<HTMLDivElement>('.live-view-msg')!;
    el.textContent = text;
    el.className = `live-view-msg ${kind === 'info' ? '' : `is-${kind}`}`;
  }

  /** Back to teaching after a run: the part, fixture and gripper again, in the cell. (Teaching: nothing to do.) */
  public close() {
    if (this.mode !== 'live') return;
    if (this.runId !== null) this.viewer.stopAnimation();
    this.clearScene();
    this.mode = 'none';
    this.overlay.classList.remove('visible', 'is-live');
    this.hideTeaching(false);
    this.viewer.requestRender();
    if (this.viewer.getModelRoot()) this.showCell();
  }

  /** Removes whatever cell is drawn (meshes, room, lights) and stops its updates. */
  private clearScene() {
    this.session++;
    if (this.pollTimer !== null) window.clearTimeout(this.pollTimer);
    if (this.raf !== null) cancelAnimationFrame(this.raf);
    this.pollTimer = this.raf = null;
    this.runId = null;
    this.scene = null;
    this.frames = [];
    this.nextFrame = 0;
    this.playT = null;
    this.framed = false;
    this.root.visible = false;
    this.root.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh) return;
      const m = mesh.material as THREE.MeshStandardMaterial;
      m.map?.dispose();
      m.dispose();
      if (!mesh.userData.sharedGeometry) mesh.geometry.dispose(); // part / table meshes stay cached for next time
    });
    this.root.clear();
    this.bodies = [];
    for (const o of this.roomLights) this.viewer.getScene().remove(o);
    this.roomLights = [];
  }

  private async build(scene: ViewScene, session: number, cell = false) {
    this.bodies = scene.bodies.map(() => {
      const g = new THREE.Group();
      this.root.add(g);
      return g;
    });
    const loads = scene.geoms.map(async (g) => {
      const geom = g.type === 'mesh' && g.mesh ? await loadMesh(g.mesh) : primitive(g);
      if (!geom) return;
      const mesh = new THREE.Mesh(geom, materialFor(g, scene.look));
      mesh.castShadow = g.body !== null || g.geomName?.startsWith('pickstation') === true;
      mesh.receiveShadow = true;
      mesh.userData.geomName = g.geomName ?? '';
      mesh.userData.sharedGeometry = g.type === 'mesh';
      mesh.raycast = () => {}; // not clickable: points are marked on the part in teaching
      setPose(mesh, [...g.pos, ...g.quat]);
      (g.body === null ? this.root : this.bodies[g.body]).add(mesh);
    });
    try {
      await Promise.all(loads);
    } catch (err) {
      console.warn('Live view:', err);
    }
    if (session !== this.session) return;
    this.scene = scene;
    this.placeRoot(cell);
    this.addRoom(scene);
    this.viewer.setLiveLook(true);
    this.root.visible = true;
    if (cell) {
      // one frame: the arm at home
      const r = await fetch('/api/cell/frames', { cache: 'no-store' });
      if (session !== this.session || !r.ok) return;
      const f = new Float32Array(await r.arrayBuffer());
      this.bodies.forEach((body, k) => {
        const o = 1 + 7 * k;
        body.position.set(f[o], f[o + 1], f[o + 2]);
        body.quaternion.set(f[o + 4], f[o + 5], f[o + 6], f[o + 3]);
      });
      this.viewer.requestRender();
      return;
    }
    this.lastWall = performance.now();
    this.poll();
    this.raf = requestAnimationFrame(this.tick);
  }

  private roomLights: THREE.Object3D[] = [];

  /** The industrial room around the table, and its lights (cellLook.ts). */
  private addRoom(scene: ViewScene) {
    const table = new THREE.Box3();
    for (const child of this.root.children) {
      const mesh = child as THREE.Mesh;
      if (!mesh.isMesh || !String(mesh.userData.geomName).startsWith('table_base')) continue;
      mesh.updateMatrix();
      mesh.geometry.computeBoundingBox();
      table.union(mesh.geometry.boundingBox!.clone().applyMatrix4(mesh.matrix));
    }
    if (table.isEmpty()) return;
    const { room, hemi, sun, target, center } = buildRoom(table, scene.floorZ ?? table.min.z);
    this.root.add(room);
    this.root.updateMatrixWorld(true);
    // lights live in the 3D view's frame (mm): the shadow camera must not be scaled
    const at = (dx: number, dy: number, dz: number) => this.root.localToWorld(center.clone().add(new THREE.Vector3(dx, dy, dz)));
    target.position.copy(at(0, 0, 0));
    sun.position.copy(at(1.4, -1.1, 2.8));
    sun.target = target;
    const cam = sun.shadow.camera;
    cam.left = cam.bottom = -1400;
    cam.right = cam.top = 1400;
    cam.near = 10;
    cam.far = 9000;
    cam.updateProjectionMatrix();
    hemi.position.set(0, 0, 1);
    const scene3 = this.viewer.getScene();
    for (const o of [hemi, sun, target]) scene3.add(o);
    this.roomLights = [hemi, sun, target];
  }

  /** Sim world (m) -> the 3D view: part frame (T_view), mm, then wherever the part sits in the view. The teaching
   * cell is fixed in the view (the part turned in Part Pose turns on the fixture, as it will in the simulation). */
  private placeRoot(cell = false) {
    const T = new THREE.Matrix4();
    const t = this.scene!.T_view;
    T.set(t[0][0], t[0][1], t[0][2], t[0][3], t[1][0], t[1][1], t[1][2], t[1][3],
      t[2][0], t[2][1], t[2][2], t[2][3], 0, 0, 0, 1);
    const S = new THREE.Matrix4().makeScale(1000, 1000, 1000);
    const part = cell ? null : this.viewer.getModelRoot();
    part?.updateMatrixWorld(true);
    this.root.matrix.copy(part ? part.matrixWorld : new THREE.Matrix4()).multiply(S).multiply(T);
    this.root.matrixWorldNeedsUpdate = true;
  }

  private poll = async () => {
    const runId = this.runId;
    const scene = this.scene;
    if (!runId || !scene) return;
    try {
      const r = await fetch(`/api/mujoco/view/frames?id=${encodeURIComponent(runId)}&from=${this.nextFrame}`, {
        cache: 'no-store'
      });
      if (r.ok && runId === this.runId) {
        const all = new Float32Array(await r.arrayBuffer());
        for (let k = 0; k + scene.frameFloats <= all.length; k += scene.frameFloats) {
          this.frames.push(all.subarray(k, k + scene.frameFloats));
          this.nextFrame++;
        }
        if (this.frames.length > 400) this.frames.splice(0, this.frames.length - 400);
      }
    } catch {
      // keep polling
    }
    if (runId === this.runId && !this.ended) this.pollTimer = window.setTimeout(this.poll, POLL_MS);
  };

  private tick = () => {
    this.raf = requestAnimationFrame(this.tick);
    const now = performance.now();
    const dt = (now - this.lastWall) / 1000;
    this.lastWall = now;
    const n = this.frames.length;
    if (!n) return;
    const newest = this.frames[n - 1][0];
    if (this.playT === null) this.playT = this.frames[0][0];
    this.playT += dt;
    if (this.ended) this.playT = Math.min(this.playT, newest);
    else if (newest - this.playT > MAX_LAG_S || this.playT > newest) this.playT = Math.max(this.frames[0][0], newest - LAG_S);
    // frames a, b around playT
    let i = n - 1;
    while (i > 0 && this.frames[i][0] > this.playT) i--;
    const a = this.frames[i];
    const b = this.frames[Math.min(i + 1, n - 1)];
    const s = b[0] > a[0] ? Math.min(1, Math.max(0, (this.playT - a[0]) / (b[0] - a[0]))) : 0;
    const qa = new THREE.Quaternion();
    const qb = new THREE.Quaternion();
    this.bodies.forEach((body, k) => {
      const o = 1 + 7 * k;
      body.position.set(
        a[o] + (b[o] - a[o]) * s,
        a[o + 1] + (b[o + 1] - a[o + 1]) * s,
        a[o + 2] + (b[o + 2] - a[o + 2]) * s
      );
      qa.set(a[o + 4], a[o + 5], a[o + 6], a[o + 3]);
      qb.set(b[o + 4], b[o + 5], b[o + 6], b[o + 3]);
      body.quaternion.slerpQuaternions(qa, qb, s);
    });
    if (!this.framed) {
      // frame the workspace: arm, gripper, part, fixture, tray, robot base (not the table, it is much bigger)
      this.framed = true;
      this.root.updateMatrixWorld(true);
      const box = new THREE.Box3();
      this.bodies.forEach((b) => box.union(new THREE.Box3().setFromObject(b)));
      for (const child of this.root.children) {
        if (!(child as THREE.Mesh).isMesh) continue;
        const b = new THREE.Box3().setFromObject(child);
        const s = b.getSize(new THREE.Vector3());
        if (Math.max(s.x, s.y, s.z) < 600) box.union(b); // mm: skip the table and floor
      }
      if (!box.isEmpty()) {
        // from in front of the robot (sim world +x, a little to the side, from above), in the view's frame
        const dir = new THREE.Vector3(1.0, -0.55, 0.75).transformDirection(this.root.matrixWorld);
        this.viewer.fitToBox(box.expandByScalar(60), true, dir);
      }
    }
    this.viewer.requestRender();
  };
}
