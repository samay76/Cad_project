/**
 * Draws a planned pick & place in the CAD viewport: the workspace (table, tray), the TCP path, and a
 * playback of the gripper carrying the part. The robot arm itself is shown by the MuJoCo preview.
 *
 * Planned poses are relative to the part's starting pose, so everything here hangs off the part's
 * current pose in the viewer (T_viewer_part0).
 */
import * as THREE from 'three';
import type { GraspController } from '../gripper/GraspController';
import type { Pose } from '../types';
import type { GraspPlan, MarkedPoint, PlanDocument } from './api';
import type { GripCue } from './gripCue';

const CUE_OK_COLOR = 0x14b8a6;
const CUE_BAD_COLOR = 0xef4444;
const CUE_RADIUS_MM = 11; // about half a finger pad's width on both grippers

export const POINT_COLORS = ['#f97316', '#a855f7'];

const PHASE_COLORS: Record<string, number> = {
  'pre-grasp': 0x3b82f6,
  approach: 0x22c55e,
  lift: 0xf97316,
  inspect: 0xa855f7,
  transfer: 0xf97316,
  place: 0xf97316,
  retreat: 0x22c55e,
  home: 0x3b82f6
};

const poseMatrix = (p: Pose) =>
  new THREE.Matrix4().compose(
    new THREE.Vector3(p.position.x, p.position.y, p.position.z),
    new THREE.Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
    new THREE.Vector3(1, 1, 1)
  );

export class AutoPickPreview {
  private group = new THREE.Group();
  private markers = new THREE.Group(); // marked points; follows the part
  private frame = new THREE.Matrix4(); // T_viewer_part0
  private animationId: number | null = null;
  private grasp: GraspController | null = null;
  private partRoot: THREE.Object3D | null = null;

  constructor(
    scene: THREE.Scene,
    private requestRender: () => void
  ) {
    this.group.matrixAutoUpdate = false;
    scene.add(this.group);
    this.markers.matrixAutoUpdate = false;
    this.markers.renderOrder = 10;
    scene.add(this.markers);
  }

  /** Marks where each finger should go, visible even through the gripper. */
  public setPoints(points: MarkedPoint[], partRoot: THREE.Object3D | null, cue: GripCue | null = null) {
    this.markers.traverse((obj) => {
      const item = obj as THREE.Mesh;
      item.geometry?.dispose();
      (item.material as THREE.Material | undefined)?.dispose?.();
    });
    this.markers.clear();
    if (partRoot) {
      const size = new THREE.Box3().setFromObject(partRoot).getSize(new THREE.Vector3());
      const radius = Math.max(1.2, 0.03 * Math.max(size.x, size.y, size.z));
      points.forEach((m, i) => {
        const sphere = new THREE.Mesh(
          new THREE.SphereGeometry(radius, 16, 16),
          new THREE.MeshBasicMaterial({ color: POINT_COLORS[i], depthTest: false, transparent: true })
        );
        sphere.renderOrder = 10;
        sphere.position.set(m.point.x, m.point.y, m.point.z);
        this.markers.add(sphere);
      });
      if (cue?.geometry) this.markers.add(this.buildGripCue(cue, radius));
      else if (cue) this.markers.add(this.buildNoGripBar(points, radius));
      this.followPart(partRoot);
    }
    this.requestRender();
  }

  /** No closing direction exists for these two points, so just a red see-through bar joining them. */
  private buildNoGripBar(points: MarkedPoint[], radius: number) {
    const [a, b] = points.map((m) => new THREE.Vector3(m.point.x, m.point.y, m.point.z));
    const bar = new THREE.Mesh(
      new THREE.CylinderGeometry(radius * 0.8, radius * 0.8, Math.max(a.distanceTo(b), 0.01), 24),
      new THREE.MeshBasicMaterial({ color: CUE_BAD_COLOR, transparent: true, opacity: 0.3, depthTest: false, depthWrite: false })
    );
    bar.renderOrder = 9;
    bar.position.copy(a).add(b).multiplyScalar(0.5);
    if (a.distanceTo(b) > 0) bar.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
    return bar;
  }

  /** See-through shape between the two finger planes, with arrows for the way the fingers move. */
  private buildGripCue(cue: GripCue, markerRadius: number) {
    const { mode, axis, center, widthMm, offsetMm } = cue.geometry!;
    const color = cue.ok ? CUE_OK_COLOR : CUE_BAD_COLOR;
    const radius = Math.max(CUE_RADIUS_MM, offsetMm / 2 + markerRadius);
    const material = (opacity: number) =>
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthTest: false, depthWrite: false, side: THREE.DoubleSide });

    const group = new THREE.Group();
    group.renderOrder = 9;
    group.position.copy(center);
    group.quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), axis);

    const volume = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, widthMm, 32, 1, true), material(0.22));
    volume.rotation.z = Math.PI / 2; // the cylinder's own axis is Y; the closing axis is local X
    volume.renderOrder = 9;
    group.add(volume);

    const arrowLength = Math.max(10, Math.min(25, radius));
    for (const side of [1, -1]) {
      const disc = new THREE.Mesh(new THREE.CircleGeometry(radius, 32), material(0.4));
      disc.rotation.y = Math.PI / 2;
      disc.position.x = (side * widthMm) / 2;
      disc.renderOrder = 9;
      group.add(disc);

      // Outside grip: from outside towards the part. Inside grip: from the plane outwards, against the wall.
      const inward = mode === 'outside';
      const dir = new THREE.Vector3(inward ? -side : side, 0, 0);
      const start = new THREE.Vector3((side * widthMm) / 2 + (inward ? side * arrowLength : 0), 0, 0);
      const arrow = new THREE.ArrowHelper(dir, start, arrowLength, color, arrowLength * 0.4, arrowLength * 0.3);
      arrow.traverse((part) => {
        const piece = part as THREE.Mesh;
        const pieceMaterial = piece.material as THREE.Material | undefined;
        if (pieceMaterial) pieceMaterial.depthTest = false;
        piece.renderOrder = 10;
      });
      group.add(arrow);
    }
    return group;
  }

  /** The part was moved in the viewer: the marked points move with it. */
  public followPart(partRoot: THREE.Object3D) {
    partRoot.updateMatrixWorld(true);
    this.markers.matrix.copy(partRoot.matrixWorld);
    this.markers.matrixWorldNeedsUpdate = true;
  }

  /** Draws the workspace and path of one plan. Returns the region to frame with the camera. */
  public show(doc: PlanDocument, plan: GraspPlan, partRoot: THREE.Object3D): THREE.Box3 {
    this.stop();
    this.clearDrawing();
    partRoot.updateMatrixWorld(true);
    this.frame.copy(partRoot.matrixWorld);
    this.group.matrix.copy(this.frame);
    this.group.matrixWorldNeedsUpdate = true;

    // Table and tray: world-frame boxes, moved into the part's starting frame.
    const workspace = new THREE.Group();
    workspace.matrixAutoUpdate = false;
    workspace.matrix.copy(poseMatrix(doc.world_in_part));
    for (const b of doc.scene.boxes) {
      const isTable = b.name === 'table_top';
      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(b.halfSize[0] * 2, b.halfSize[1] * 2, b.halfSize[2] * 2),
        new THREE.MeshStandardMaterial({
          color: isTable ? 0xd6c2a1 : 0x475569,
          transparent: true,
          opacity: isTable ? 0.35 : 0.8,
          depthWrite: !isTable
        })
      );
      mesh.position.set(b.pos[0], b.pos[1], b.pos[2]);
      workspace.add(mesh);
    }
    this.group.add(workspace);

    // TCP path, colored by what the arm is doing (blue free space, green near the part, orange carrying).
    for (const phase of plan.phases) {
      if (phase.tcp.length < 2) continue;
      const points = phase.tcp.map((p) => new THREE.Vector3(p.position.x, p.position.y, p.position.z));
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(points),
        new THREE.LineBasicMaterial({ color: PHASE_COLORS[phase.name] ?? 0x64748b })
      );
      this.group.add(line);
    }

    // Region for the camera: the table is far larger than the motion, so frame the motion and the part.
    this.group.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(partRoot);
    const pt = new THREE.Vector3();
    for (const phase of plan.phases) {
      for (const p of phase.tcp) {
        box.expandByPoint(pt.set(p.position.x, p.position.y, p.position.z).applyMatrix4(this.frame));
      }
    }
    this.requestRender();
    return box;
  }

  /** Plays the motion once; onDone runs after the gripper and part are back where they were. */
  public play(plan: GraspPlan, grasp: GraspController, partRoot: THREE.Object3D, onPhase: (label: string) => void, onDone: () => void) {
    this.stop();
    this.grasp = grasp;
    this.partRoot = partRoot;
    grasp.beginExternalMotion();

    const T_tcp_part = new THREE.Matrix4().set(...(plan.T_tcp_part.flat().map((v, i) =>
      i % 4 === 3 && i < 12 ? v * 1000 : v
    ) as Parameters<THREE.Matrix4['set']>));
    const placedRelease = plan.phases.findIndex((ph) => ph.name === 'release');

    const start = performance.now();
    const total = plan.phases.reduce((sum, ph) => sum + ph.seconds, 0);
    let width = plan.phases[0].width;
    let partPlaced: THREE.Matrix4 | null = null;
    let shownPhase = -1;

    const T_tcp = new THREE.Matrix4();
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const qa = new THREE.Quaternion();
    const qb = new THREE.Quaternion();
    const scale = new THREE.Vector3();

    const tick = (now: number) => {
      const elapsed = (now - start) / 1000;
      if (elapsed >= total) {
        this.stop();
        onDone();
        return;
      }
      let t = elapsed;
      let index = 0;
      while (index < plan.phases.length - 1 && t >= plan.phases[index].seconds) {
        // Width and part state carried from one phase to the next.
        width = plan.phases[index].name === 'grip' ? gripWidth(plan) : plan.phases[index].width;
        t -= plan.phases[index].seconds;
        index++;
      }
      const phase = plan.phases[index];
      if (index !== shownPhase) {
        shownPhase = index;
        onPhase(phase.label);
      }
      const s = 0.5 * (1 - Math.cos((Math.PI * t) / phase.seconds));
      const u = s * (phase.tcp.length - 1);
      const k = Math.min(Math.floor(u), Math.max(phase.tcp.length - 2, 0));
      const a = phase.tcp[k];
      const b = phase.tcp[Math.min(k + 1, phase.tcp.length - 1)];
      const f = u - k;
      pos.set(
        a.position.x + (b.position.x - a.position.x) * f,
        a.position.y + (b.position.y - a.position.y) * f,
        a.position.z + (b.position.z - a.position.z) * f
      );
      qa.set(a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w);
      qb.set(b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w);
      quat.slerpQuaternions(qa, qb, f);
      T_tcp.compose(pos, quat, scale.set(1, 1, 1)).premultiply(this.frame);

      const endWidth = phase.name === 'grip' ? gripWidth(plan) : phase.width;
      const widthNow = width + (endWidth - width) * s;
      T_tcp.decompose(pos, quat, scale);
      grasp.applyExternalMotion(pos, quat, widthNow);

      if (phase.attached) {
        const T_part = T_tcp.clone().multiply(T_tcp_part);
        T_part.decompose(partRoot.position, partRoot.quaternion, scale);
        this.followPart(partRoot);
        partPlaced = T_part;
      } else if (partPlaced && index >= placedRelease) {
        partPlaced.decompose(partRoot.position, partRoot.quaternion, scale);
        this.followPart(partRoot);
      }
      this.requestRender();
      this.animationId = requestAnimationFrame(tick);
    };
    this.animationId = requestAnimationFrame(tick);
  }

  /** Ends a playback and puts the gripper and part back. */
  public stop() {
    if (this.animationId !== null) {
      cancelAnimationFrame(this.animationId);
      this.animationId = null;
    }
    if (this.grasp) {
      this.grasp.stopTestGrasp();
      this.grasp = null;
      if (this.partRoot) this.followPart(this.partRoot);
      this.partRoot = null;
      this.requestRender();
    }
  }

  public get playing() {
    return this.grasp !== null;
  }

  public clear() {
    this.stop();
    this.clearDrawing();
    this.requestRender();
  }

  private clearDrawing() {
    this.group.traverse((obj) => {
      const item = obj as THREE.Mesh;
      item.geometry?.dispose();
      (item.material as THREE.Material | undefined)?.dispose?.();
    });
    this.group.clear();
  }
}

/** Opening the fingers end at when gripping: on the part, or against the walls for an inside grip. */
export const gripWidth = (plan: GraspPlan) =>
  plan.mode === 'outside' ? plan.widthMm : plan.phases.find((ph) => ph.name === 'lift')!.width;
