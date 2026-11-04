/**
 * Runs the automatic pick & place workflow in the web app:
 *   Load STEP -> mark two points, one per finger -> plan -> preview -> Execute.
 * Grasp and motion planning happen in Python (jenga_mujoco/auto_pick_place.py), reached through the
 * dev-server bridge. This class keeps the workflow state and connects the panel, viewer and bridge.
 */
import * as THREE from 'three';
import type { GraspController } from '../gripper/GraspController';
import type { AutoPickPanel, AutoPickView } from '../ui/AutoPickPanel';
import type { CameraFeedPanel } from '../ui/CameraFeedPanel';
import type { ClickedPointInfo, SavedGrasp } from '../types';
import type { Viewer } from '../viewer/Viewer';
import { GRIPPERS, type GripperId } from '../config';
import { AutoPickPreview, gripWidth, POINT_COLORS } from './AutoPickPreview';
import { arrayBufferToBase64, fetchPlan, fetchStatus, startPlan, startRun, type MarkedPoint, type PlanDocument, type SimStatus } from './api';
import { describeGrip } from './gripCue';

export interface AutoPickDeps {
  viewer: Viewer;
  getGraspController: () => GraspController | null;
  getPart: () => { name: string; buffer: ArrayBuffer | null };
  getGripperId: () => GripperId;
  getCameraPanel: () => CameraFeedPanel | null;
  getRobotId: () => string;
  restoreGrasp: (grasp: SavedGrasp) => Promise<void>;
  showError: (message: string, title: string) => void;
}

const POLL_MS = 500;
const ORIENTATION_TOLERANCE_DEG = 0.5;
const MIN_POINT_DISTANCE_MM = 2;

export class AutoPickController {
  private preview: AutoPickPreview;
  private doc: PlanDocument | null = null;
  private points: MarkedPoint[] = [];
  private runId: string | null = null;
  private selected = 0;
  private busy = false;
  private status: AutoPickView['status'] = null;
  private plannedQuaternion = new THREE.Quaternion();
  private pollTimer: number | null = null;

  constructor(
    private panel: AutoPickPanel,
    private deps: AutoPickDeps
  ) {
    this.preview = new AutoPickPreview(deps.viewer.getScene(), () => deps.viewer.requestRender());
    panel.setEvents({
      onCalculate: () => this.calculate(),
      onClearPoints: () => this.clearPoints(),
      onSelectGrasp: (i) => this.selectGrasp(i),
      onPreviewHere: () => this.togglePreviewHere(),
      onPreviewRobot: () => this.runInMujoco('preview'),
      onExecute: () => this.runInMujoco('execute')
    });
    this.refresh();
  }

  /** The part, gripper or robot changed: a plan made for the old one no longer applies. */
  public invalidate() {
    this.stopPolling();
    this.preview.clear();
    this.doc = null;
    this.runId = null;
    this.selected = 0;
    this.busy = false;
    this.status = null;
    this.preview.setPoints(this.points, this.partRoot(), this.gripCue());
    this.refresh();
  }

  /** The part was moved in the viewer: the marked points move with it. */
  public partMoved() {
    const root = this.partRoot();
    if (root && !this.preview.playing) this.preview.followPart(root);
  }

  private gripCue() {
    return describeGrip(this.points, GRIPPERS[this.deps.getGripperId()].maxOpeningWidth);
  }

  /** A click on the part marks the next finger's point. A click after both are set starts over. */
  public markPoint(click: ClickedPointInfo) {
    if (!click.fromSurfaceClick || this.busy || this.preview.playing) return;
    if (this.points.length === 2) this.points = [];
    this.points.push({ point: click.point, normal: click.normal });
    this.invalidate();
  }

  public clearPoints() {
    this.points = [];
    this.invalidate();
  }

  public refresh() {
    const { name, buffer } = this.deps.getPart();
    this.panel.render({
      partName: buffer ? name : null,
      points: this.points.map((m) => `(${m.point.x.toFixed(1)}, ${m.point.y.toFixed(1)}, ${m.point.z.toFixed(1)}) mm`),
      pointColors: POINT_COLORS,
      gripHint: this.gripCue(),
      busy: this.busy,
      playing: this.preview.playing,
      status: this.status,
      selected: this.selected,
      grasps: (this.doc?.plans ?? []).map((plan, i) => ({
        title: `Grasp ${i + 1}: ${plan.mode} grip, ${plan.widthMm.toFixed(1)} mm`,
        detail:
          `Approach ${plan.approachTiltDeg.toFixed(0)}° from vertical, ${plan.durationS.toFixed(0)} s motion` +
          (plan.inspectionViews ? `, ${plan.inspectionViews} camera views` : '')
      }))
    });
  }

  private setStatus(text: string, kind: 'info' | 'ok' | 'error' = 'info') {
    this.status = { text, kind };
    this.refresh();
  }

  private partRoot() {
    return this.deps.viewer.getModelRoot();
  }

  // ---------------- calculate ----------------
  private async calculate() {
    const { name, buffer } = this.deps.getPart();
    const root = this.partRoot();
    const grasp = this.deps.getGraspController();
    if (!buffer || !root) return this.fail('Load a STEP file first.');
    if (!/\.(stp|step)$/i.test(name)) {
      return this.fail('Automatic pick & place needs a STEP file (.step or .stp): the simulator converts its exact geometry.');
    }
    if (this.points.length !== 2) return this.fail('Mark two points on the part, one for each finger.');
    const [a, b] = this.points.map((m) => new THREE.Vector3(m.point.x, m.point.y, m.point.z));
    if (a.distanceTo(b) < MIN_POINT_DISTANCE_MM) return this.fail('The two points are in the same place. Mark one point for each finger.');
    if (!grasp) return this.fail('The gripper is still loading. Try again in a moment.');

    this.preview.clear();
    this.doc = null;
    this.selected = 0;
    this.busy = true;
    this.setStatus('Starting the planner (the first run of a part converts its mesh, ~30 s)...');

    root.updateMatrixWorld(true);
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    root.matrixWorld.decompose(pos, quat, new THREE.Vector3());
    this.plannedQuaternion.copy(quat);
    const state = grasp.getCurrentState();

    try {
      const runId = await startPlan({
        partFileName: name,
        partFileBase64: arrayBufferToBase64(buffer),
        partWorldPose: {
          position: { x: pos.x, y: pos.y, z: pos.z },
          orientation: { x: quat.x, y: quat.y, z: quat.z, w: quat.w }
        },
        robotId: this.deps.getRobotId(),
        gripperId: this.deps.getGripperId(),
        tcpOffset: state.tcpOffset,
        preGraspDistance: state.preGraspDistance,
        points: this.points
      });
      this.runId = runId;
      this.watch(runId, async (ok, message) => {
        if (!ok) return this.fail(message);
        this.doc = await fetchPlan(runId);
        this.busy = false;
        const tipped = this.doc.part.tippedDeg > 5
          ? ` The part tips over ${this.doc.part.tippedDeg.toFixed(0)}° when released on the table, so the table is drawn tilted here.`
          : '';
        this.setStatus(message + tipped, 'ok');
        await this.showSelected();
      });
    } catch (err: any) {
      this.fail(`Could not start the planner: ${err?.message || err}. Is the app running via "npm run dev"?`);
    }
  }

  private fail(message: string) {
    this.busy = false;
    this.setStatus(message, 'error');
    this.deps.showError(message, 'Auto Pick & Place');
  }

  // ---------------- show & preview ----------------
  private async selectGrasp(index: number) {
    this.selected = index;
    this.refresh();
    await this.showSelected();
  }

  private async showSelected() {
    const root = this.partRoot();
    const plan = this.doc?.plans[this.selected];
    if (!root || !plan) return;
    this.deps.viewer.fitToBox(this.preview.show(this.doc!, plan, root));
    await this.deps.restoreGrasp(plan.grasp);
    // Show the fingers on the marked points. The viewer's collision badge uses bounding boxes for the 2F-85
    // linkage and can flag this grip even though the planner checked it against the real shapes.
    this.deps.getGraspController()?.setOpeningWidth(gripWidth(plan));
    this.refresh();
  }

  /** True if the part was turned since planning; the sim places it on the table in the orientation shown. */
  private planIsStale() {
    const root = this.partRoot();
    if (!root) return true;
    root.updateMatrixWorld(true);
    const quat = new THREE.Quaternion();
    root.matrixWorld.decompose(new THREE.Vector3(), quat, new THREE.Vector3());
    if (THREE.MathUtils.radToDeg(quat.angleTo(this.plannedQuaternion)) <= ORIENTATION_TOLERANCE_DEG) return false;
    this.setStatus('The part was rotated after planning. Press Calculate Grasp again.', 'error');
    return true;
  }

  private togglePreviewHere() {
    if (this.preview.playing) {
      this.preview.stop();
      this.refresh();
      return;
    }
    const grasp = this.deps.getGraspController();
    const root = this.partRoot();
    const plan = this.doc?.plans[this.selected];
    if (!grasp || !root || !plan || this.planIsStale()) return;
    this.preview.play(
      plan,
      grasp,
      root,
      (label) => this.setStatus(`Preview: ${label}...`),
      () => this.setStatus('Preview finished. Press Execute Pick & Place to run it with physics.', 'ok')
    );
    this.refresh();
  }

  private async runInMujoco(mode: 'preview' | 'execute') {
    const plan = this.doc?.plans[this.selected];
    if (!this.runId || !plan || this.planIsStale()) return;
    this.busy = true;
    this.setStatus(mode === 'preview' ? 'Opening the MuJoCo preview...' : 'Starting the simulation...');
    try {
      const runId = await startRun(this.runId, this.selected, mode);
      const camera = mode === 'execute' ? this.deps.getCameraPanel() : null;
      camera?.start(runId);
      this.watch(
        runId,
        (ok, message) => {
          this.busy = false;
          camera?.finish(message);
          if (ok) this.setStatus(message, 'ok');
          else this.fail(message);
        },
        (st) => {
          if (st.info?.inspection) camera?.setImages(st.info.inspection);
          if (st.state === 'running' && st.message) camera?.setCaption(st.message);
        }
      );
    } catch (err: any) {
      this.fail(`Could not start MuJoCo: ${err?.message || err}.`);
    }
  }

  // ---------------- status polling ----------------
  private watch(runId: string, onEnd: (ok: boolean, message: string) => void, onUpdate?: (st: SimStatus) => void) {
    this.stopPolling();
    let lastMessage = '';
    this.pollTimer = window.setInterval(async () => {
      let st;
      try {
        st = await fetchStatus(runId);
      } catch {
        return; // dev server restarting; keep polling
      }
      if (this.pollTimer === null) return; // stopped while the request was in flight
      onUpdate?.(st);
      if (st.state === 'running') {
        if (st.message && st.message !== lastMessage) {
          lastMessage = st.message;
          this.setStatus(st.message);
        }
        return;
      }
      this.stopPolling();
      onEnd(!!st.ok, st.message || 'The simulator ended without a message (see the terminal running npm run dev).');
    }, POLL_MS);
  }

  private stopPolling() {
    if (this.pollTimer !== null) window.clearInterval(this.pollTimer);
    this.pollTimer = null;
  }
}
