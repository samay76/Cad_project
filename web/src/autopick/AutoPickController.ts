/**
 * Runs the automatic pick & place workflow in the web app:
 *   Load STEP / mesh -> mark two points, one per finger -> plan -> preview -> Sim -> Real Sim.
 * Real Sim (cell/real_robot.py) moves the real EC66 through the planned joints; it is enabled only once the
 * Sim of the same plan and grasp has run to the end.
 * Grasp and motion planning happen in Python (cell/auto_pick_place.py), reached through the
 * cell server (python -m server). This class keeps the workflow state and connects the panel, viewer and server.
 */
import * as THREE from 'three';
import type { GraspController } from '../gripper/GraspController';
import type { AutoPickPanel, AutoPickView } from '../ui/AutoPickPanel';
import type { ClickedPointInfo, Pose, SavedGrasp } from '../types';
import type { Viewer } from '../viewer/Viewer';
import { GRIPPERS, type GripperId } from '../config';
import { LiveView } from './LiveView';
import { AutoPickPreview, gripWidth, POINT_COLORS } from './AutoPickPreview';
import {
  arrayBufferToBase64,
  fetchPlan,
  fetchStatus,
  startPlan,
  startRealRun,
  startRun,
  stopRun,
  type MarkedPoint,
  type PlanDocument,
  type RealRobotSettings,
  type SimStatus
} from './api';
import { describeGrip } from './gripCue';

export interface AutoPickDeps {
  viewer: Viewer;
  getGraspController: () => GraspController | null;
  getPart: () => { name: string; buffer: ArrayBuffer | null };
  getGripperId: () => GripperId;
  getRobotId: () => string;
  restoreGrasp: (grasp: SavedGrasp) => Promise<void>;
  showError: (message: string, title: string) => void;
}

const POLL_MS = 500;
const ORIENTATION_TOLERANCE_DEG = 0.5;
const MIN_POINT_DISTANCE_MM = 2;
/** Part files the simulator converts (cell/step_to_mesh.py). Meshes are in mm, like the viewer shows them. */
export const SIM_PART_FILE = /\.(stp|step|stl|obj)$/i;
const REAL_SETTINGS_KEY = 'autopick.realRobot';
const PICK_OFFSET_KEY = 'autopick.pickOffset';
const MODE_KEY = 'autopick.mode';
type PickOffset = { x: number; y: number; z: number };

function loadPickOffset(): PickOffset {
  try {
    const saved = JSON.parse(localStorage.getItem(PICK_OFFSET_KEY) || '{}');
    return { x: Number(saved.x) || 0, y: Number(saved.y) || 0, z: Number(saved.z) || 0 };
  } catch {
    return { x: 0, y: 0, z: 0 };
  }
}
const DEFAULT_REAL: RealRobotSettings = { ip: '192.168.1.200', src: '', speed: 5, gripper: true };

function loadRealSettings(): RealRobotSettings {
  try {
    const saved = JSON.parse(localStorage.getItem(REAL_SETTINGS_KEY) || '{}');
    return { ...DEFAULT_REAL, ...saved };
  } catch {
    return { ...DEFAULT_REAL };
  }
}

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
  private real = loadRealSettings();
  private pickOffset = loadPickOffset();
  private plannedOffset: PickOffset | null = null; // the offset the current plan was made with
  private shownGraspPos: THREE.Vector3 | null = null; // gripper position (part frame, mm) right after showing the plan
  private jogFrame: number[][] | null = null; // T_world_part of that plan, kept while the points are re-marked
  private shownGraspQuat: THREE.Quaternion | null = null; // and its orientation
  private taughtPose: Pose | null = null; // gripper pose set with Step Jogging: every plan uses it until new points
  private jogPressed = false; // a Step Jogging button was pressed since the plan was shown (not just a tiny drift)
  /** `${runId}:${candidate}` whose Sim ran to the end: Real Sim is allowed for exactly that plan and grasp. */
  private simChecked: string | null = null;
  private processRunning = false;
  private live: LiveView; // Sim / Real Sim shown live in the 3D view
  /** grip: two marked points, one per finger; touch: the closed fingertips go to one marked point */
  private mode: 'grip' | 'touch' = (() => {
    try {
      return localStorage.getItem(MODE_KEY) === 'touch' ? 'touch' : 'grip';
    } catch {
      return 'grip';
    }
  })();

  constructor(
    private panel: AutoPickPanel,
    private deps: AutoPickDeps
  ) {
    this.preview = new AutoPickPreview(deps.viewer.getScene(), () => deps.viewer.requestRender());
    this.live = new LiveView(deps.viewer, (hide) => this.hideTeaching(hide));
    panel.setEvents({
      onCalculate: () => this.calculate(),
      onClearPoints: () => this.clearPoints(),
      onSelectGrasp: (i) => this.selectGrasp(i),
      onPreviewHere: () => this.togglePreviewHere(),
      onPreviewRobot: () => this.runInMujoco('preview'),
      onExecute: () => this.runInMujoco('execute'),
      onRealRun: () => this.runOnRealRobot(),
      onGoHome: () => this.goHome(),
      onStop: () => this.stop(),
      onRobotSettings: (settings) => this.setRealSettings(settings),
      onPickOffset: (offset) => this.setPickOffset(offset),
      onMode: (mode) => this.setMode(mode)
    });
    this.refresh();
  }

  /** While the live cell is shown: the teaching part, fixture, gripper and plan drawing are hidden. */
  private hideTeaching(hide: boolean) {
    const root = this.deps.viewer.getModelRoot();
    if (root) root.visible = !hide;
    const fixture = this.deps.viewer.getScene().getObjectByName('pick_fixture');
    if (fixture) fixture.visible = !hide;
    this.deps.getGraspController()?.setVisible(!hide);
    this.preview.setVisible(!hide);
    this.deps.viewer.requestRender();
  }

  /** The part, gripper or robot changed: a plan made for the old one no longer applies. */
  public invalidate() {
    this.stopPolling();
    this.preview.clear();
    this.doc = null;
    this.shownGraspPos = null;
    this.jogFrame = null;
    this.jogPressed = false;
    this.runId = null;
    this.selected = 0;
    this.busy = false;
    this.status = null;
    this.simChecked = null;
    this.preview.setPoints(this.points, this.partRoot(), this.gripCue());
    this.refresh();
  }

  /** The part was moved in the viewer: the marked points move with it. */
  public partMoved() {
    const root = this.partRoot();
    if (root && !this.preview.playing) this.preview.followPart(root);
  }

  private gripCue() {
    if (this.mode === 'touch') return null;
    return describeGrip(this.points, GRIPPERS[this.deps.getGripperId()].maxOpeningWidth);
  }

  /** A click on the part marks the next finger's point. A click after both are set starts over. */
  public markPoint(click: ClickedPointInfo) {
    if (this.live.active) return; // clicks in the live cell do not mark points
    if (!click.fromSurfaceClick || this.busy || this.preview.playing) return;
    if (this.points.length === (this.mode === 'touch' ? 1 : 2)) this.points = [];
    this.points.push({ point: click.point, normal: click.normal });
    this.resetOffset();
    this.invalidate();
  }

  private setMode(mode: 'grip' | 'touch') {
    if (mode === this.mode || this.busy) return;
    this.mode = mode;
    try {
      localStorage.setItem(MODE_KEY, mode);
    } catch {
      // ignore storage errors
    }
    this.clearPoints();
  }

  public clearPoints() {
    this.live.close();
    this.points = [];
    this.resetOffset();
    this.invalidate();
  }

  /** New points = a new pick spot: a jog or offset made for the old one no longer applies. */
  private resetOffset() {
    this.taughtPose = null;
    this.setPickOffset({ x: 0, y: 0, z: 0 });
    this.plannedOffset = null;
  }

  /** The gripper was moved in the viewer (Step Jogging): show the move and that it needs a new plan. */
  public gripperMoved() {
    this.jogPressed = true;
    if (!this.busy) this.refresh();
  }

  public refresh() {
    const { name, buffer } = this.deps.getPart();
    this.panel.render({
      partName: buffer ? name : null,
      mode: this.mode,
      points: this.points.map((m) => `(${m.point.x.toFixed(1)}, ${m.point.y.toFixed(1)}, ${m.point.z.toFixed(1)}) mm`),
      pointColors: POINT_COLORS,
      gripHint: this.gripCue(),
      busy: this.busy,
      playing: this.preview.playing,
      status: this.status,
      selected: this.selected,
      pickOffset: this.pickOffset,
      offsetHint: this.jogHint() ?? (this.offsetChanged() ? 'The pick offset changed after planning. Press Plan to Points again.' : null),
      real: { ...this.real, ...this.realState(), running: this.processRunning },
      grasps: (this.doc?.plans ?? []).map((plan, i) => ({
        title: plan.touch
          ? `Touch ${i + 1}: gripper turned ${plan.touch.rollDeg}°, stops ${plan.touch.standoffMm} mm before`
          : `Grasp ${i + 1}: ${plan.mode} grip, ${plan.widthMm.toFixed(1)} mm`,
        detail:
          `Approach ${plan.approachTiltDeg.toFixed(0)}° from vertical, ${plan.durationS.toFixed(0)} s motion` +
          (plan.graspTest ? ' (grasp test)' : '')
      }))
    });
  }

  private simKey() {
    return this.runId ? `${this.runId}:${this.selected}` : null;
  }

  private realState(): { enabled: boolean; homeEnabled: boolean; hint: string } {
    const homeEnabled = !!this.doc && this.doc.robot === 'ec66';
    if (this.doc && this.doc.robot && this.doc.robot !== 'ec66') {
      return { enabled: false, homeEnabled, hint: `This plan is for the ${this.doc.robotLabel}. Real Sim drives the Elite EC66: choose it and plan again.` };
    }
    if (!this.simChecked || this.simChecked !== this.simKey()) {
      return { enabled: false, homeEnabled, hint: 'Run Sim for this grasp and let it finish to enable Real Sim. Go Home brings the robot to the home pose Real Sim starts from.' };
    }
    const who = this.real.ip === 'fake' ? 'a simulated robot (fake)' : `the robot at ${this.real.ip}`;
    const touching = !!this.doc?.plans[this.selected]?.touch;
    const grip = !this.real.gripper
      ? 'The gripper moves in the sim only.'
      : touching
        ? 'The real gripper closes before the arm moves (closed fingertips go to the point).'
        : 'The real gripper closes at the grasp and opens at the drop.';
    return {
      enabled: true,
      homeEnabled,
      hint: `Sim checked. Real Sim moves ${who} through the same joints at ${this.real.speed}% speed (pendant in REMOTE mode). ${grip}`
    };
  }

  private setRealSettings(settings: Partial<RealRobotSettings>) {
    if (settings.speed !== undefined) settings.speed = Math.round(Math.min(10, Math.max(1, settings.speed || 5)));
    this.real = { ...this.real, ...settings };
    try {
      localStorage.setItem(REAL_SETTINGS_KEY, JSON.stringify(this.real));
    } catch {
      // storage unavailable: keep the settings for this session only
    }
    // No re-render while typing (it would move the cursor out of the field); the next status update shows it.
    // The checkbox is not typed into: re-render so the hint says what Real Sim will do with the gripper.
    if (settings.gripper !== undefined) this.refresh();
  }

  private setPickOffset(offset: Partial<PickOffset>) {
    for (const k of ['x', 'y', 'z'] as const) {
      if (offset[k] !== undefined) this.pickOffset[k] = Math.min(50, Math.max(-50, offset[k]!));
    }
    try {
      localStorage.setItem(PICK_OFFSET_KEY, JSON.stringify(this.pickOffset));
    } catch {
      // storage unavailable: keep it for this session only
    }
    // No re-render while typing (it would move the cursor out of the field).
  }

  /** How far the gripper was moved (Step Jogging) since the plan was shown: robot base frame, mm; null if not. */
  /** How far the gripper was moved / turned (Step Jogging) since the plan was shown: the move in the robot base
   * frame (mm) and the turn (deg); null if it was not. */
  private gripperJog(): (PickOffset & { turnDeg: number }) | null {
    const grasp = this.deps.getGraspController();
    const T = this.doc?.part.T_world_part ?? this.jogFrame;
    if (!this.jogPressed || !grasp || !T || !this.shownGraspPos || this.preview.playing) return null;
    const pose = grasp.getCurrentState().relativePose;
    const p = pose.position;
    const o = pose.orientation;
    const d = new THREE.Vector3(p.x, p.y, p.z).sub(this.shownGraspPos); // part frame
    const turn = this.shownGraspQuat
      ? THREE.MathUtils.radToDeg(new THREE.Quaternion(o.x, o.y, o.z, o.w).angleTo(this.shownGraspQuat))
      : 0;
    if (d.length() < 0.2 && turn < 0.5) return null;
    // part frame -> sim world (robot base) frame, with the part's rotation in the plan
    const w = [0, 1, 2].map((r) => T[r][0] * d.x + T[r][1] * d.y + T[r][2] * d.z);
    const round = (v: number) => Math.round(v * 10) / 10;
    return { x: round(w[0]), y: round(w[1]), z: round(w[2]), turnDeg: round(turn) };
  }

  private jogHint() {
    const j = this.gripperJog();
    return j
      ? `Gripper moved with Step Jogging by X ${j.x}, Y ${j.y}, Z ${j.z} mm (robot base)${j.turnDeg ? `, turned ${j.turnDeg}°` : ''}. Press Plan to Points: the arm plans a new path to exactly this gripper pose.`
      : null;
  }

  private offsetChanged() {
    const p = this.plannedOffset;
    return !!this.doc && !!p && (['x', 'y', 'z'] as const).some((k) => Math.abs(p[k] - this.pickOffset[k]) > 1e-6);
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
    this.live.close(); // back to teaching: the new plan is shown on the part
    const { name, buffer } = this.deps.getPart();
    const root = this.partRoot();
    const grasp = this.deps.getGraspController();
    if (!buffer || !root) return this.fail('Load a CAD file first.');
    if (!SIM_PART_FILE.test(name)) {
      return this.fail('Automatic pick & place needs a STEP (.step/.stp) or mesh (.stl/.obj, in mm) part.');
    }
    const touch = this.mode === 'touch';
    if (touch && this.points.length !== 1) return this.fail('Mark the point the fingertips should go to.');
    if (!touch) {
      if (this.points.length !== 2) return this.fail('Mark two points on the part, one for each finger.');
      const [a, b] = this.points.map((m) => new THREE.Vector3(m.point.x, m.point.y, m.point.z));
      if (a.distanceTo(b) < MIN_POINT_DISTANCE_MM) return this.fail('The two points are in the same place. Mark one point for each finger.');
    }
    if (!grasp) return this.fail('The gripper is still loading. Try again in a moment.');

    const jogged = this.gripperJog(); // read before the plan is cleared (calculate() applies it below)
    if (this.doc) this.jogFrame = this.doc.part.T_world_part;
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
    // Gripper moved with Step Jogging after the last plan: plan with it there (added to the pick offset).
    const jog = jogged;
    if (jog) {
      // The gripper pose set in the view is planned exactly (position and orientation, part frame), and kept for
      // the next plans of these points. Further jogs are measured from here (also if this plan fails).
      const rp = state.relativePose;
      // The marked points move with the gripper: the same spots on the finger pads, now on the part where the
      // jogged gripper really closes. The panel, the markers and the plan then show the new pick location.
      if (this.shownGraspPos && this.shownGraspQuat) {
        const before = new THREE.Matrix4().compose(this.shownGraspPos, this.shownGraspQuat, new THREE.Vector3(1, 1, 1));
        const after = new THREE.Matrix4().compose(
          new THREE.Vector3(rp.position.x, rp.position.y, rp.position.z),
          new THREE.Quaternion(rp.orientation.x, rp.orientation.y, rp.orientation.z, rp.orientation.w),
          new THREE.Vector3(1, 1, 1)
        );
        this.points = this.movePointsWithGripper(after.multiply(before.invert()), root);
        this.preview.setPoints(this.points, root, this.gripCue());
      }
      this.taughtPose = {
        position: { ...rp.position },
        orientation: { ...rp.orientation }
      };
      this.shownGraspPos = new THREE.Vector3(rp.position.x, rp.position.y, rp.position.z);
      this.shownGraspQuat = new THREE.Quaternion(rp.orientation.x, rp.orientation.y, rp.orientation.z, rp.orientation.w);
      this.jogPressed = false;
      this.setPickOffset({ x: 0, y: 0, z: 0 });
      this.setStatus(
        `Planning the arm path to the gripper pose you set (moved ${jog.x}, ${jog.y}, ${jog.z} mm` +
          `${jog.turnDeg ? `, turned ${jog.turnDeg}°` : ''})...`
      );
    }

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
        points: this.points,
        mode: this.mode,
        pickOffsetMm: { ...this.pickOffset },
        graspPose: touch ? null : this.taughtPose
      });
      this.plannedOffset = { ...this.pickOffset };
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
      this.fail(`Could not start the planner: ${err?.message || err}. Is the cell server running (python -m server)?`);
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

  /** Moves the marked points by the gripper's jog `M` (part frame, mm) and puts each back on the part's surface:
   * along its new normal, the nearest surface within 15 mm (else the moved point as it is). */
  private movePointsWithGripper(M: THREE.Matrix4, root: THREE.Object3D): MarkedPoint[] {
    const rot = new THREE.Matrix3().setFromMatrix4(M);
    const toWorld = root.matrixWorld;
    const toPart = toWorld.clone().invert();
    const caster = new THREE.Raycaster();
    return this.points.map((m) => {
      const p = new THREE.Vector3(m.point.x, m.point.y, m.point.z).applyMatrix4(M);
      const n = new THREE.Vector3(m.normal.x, m.normal.y, m.normal.z).applyMatrix3(rot).normalize();
      const pw = p.clone().applyMatrix4(toWorld);
      const nw = n.clone().transformDirection(toWorld);
      caster.set(pw.clone().addScaledVector(nw, 15), nw.clone().negate());
      caster.far = 30;
      const hit = caster.intersectObject(root, true).find((h) => (h.object as THREE.Mesh).isMesh && h.face);
      if (!hit) return { point: { x: p.x, y: p.y, z: p.z }, normal: { x: n.x, y: n.y, z: n.z } };
      const q = hit.point.clone().applyMatrix4(toPart);
      const fn = hit.face!.normal.clone().transformDirection(hit.object.matrixWorld).transformDirection(toPart);
      if (fn.dot(n) < 0) fn.negate();
      return { point: { x: q.x, y: q.y, z: q.z }, normal: { x: fn.x, y: fn.y, z: fn.z } };
    });
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
    // Where the gripper is now: Step Jogging away from here moves the pick position at the next plan.
    const p = this.deps.getGraspController()?.getCurrentState().relativePose.position;
    this.shownGraspPos = p ? new THREE.Vector3(p.x, p.y, p.z) : null;
    const q = this.deps.getGraspController()?.getCurrentState().relativePose.orientation;
    this.shownGraspQuat = q ? new THREE.Quaternion(q.x, q.y, q.z, q.w) : null;
    this.jogPressed = false;
    this.refresh();
  }

  /** True if the part was turned since planning; the sim places it on the table in the orientation shown. */
  private planIsStale() {
    const root = this.partRoot();
    if (!root) return true;
    root.updateMatrixWorld(true);
    const quat = new THREE.Quaternion();
    root.matrixWorld.decompose(new THREE.Vector3(), quat, new THREE.Vector3());
    if (this.gripperJog()) {
      this.setStatus('The gripper was moved after planning (Step Jogging). Press Plan to Points to plan the pick at the new position.', 'error');
      return true;
    }
    if (this.offsetChanged()) {
      this.setStatus('The pick offset changed after planning. Press Plan to Points again.', 'error');
      return true;
    }
    if (THREE.MathUtils.radToDeg(quat.angleTo(this.plannedQuaternion)) <= ORIENTATION_TOLERANCE_DEG) return false;
    this.setStatus('The part was rotated after planning. Press Calculate Grasp again.', 'error');
    return true;
  }

  private togglePreviewHere() {
    this.live.close();
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
      const key = this.simKey();
      const runId = await startRun(this.runId, this.selected, mode);
      this.processRunning = true;
      this.live.start(runId, mode === 'preview' ? 'Preview: planned motion, no physics' : 'Sim: MuJoCo physics');
      this.watch(
        runId,
        (ok, message, st) => {
          this.busy = false;
          this.live.finish(ok, message);
          // The whole motion was played (even if the part missed the tray): the user has checked it.
          if (mode === 'execute' && st?.state === 'done' && key === this.simKey()) this.simChecked = key;
          if (ok) this.setStatus(message + (this.simChecked === key ? ' Real Sim is now enabled.' : ''), 'ok');
          else this.fail(message);
        },
        (st) => st.state === 'running' && st.message && this.live.setMessage(st.message)
      );
    } catch (err: any) {
      this.fail(`Could not start MuJoCo: ${err?.message || err}.`);
    }
  }

  // ---------------- real robot ----------------
  private async runOnRealRobot() {
    const plan = this.doc?.plans[this.selected];
    if (!this.runId || !plan || this.planIsStale() || !this.realState().enabled) return;
    const { ip, src, speed, gripper } = this.real;
    const fake = ip === 'fake';
    const ok = window.confirm(
      fake
        ? `Run the checked motion on a SIMULATED robot (ip "fake") at ${speed}% speed?`
        : `MOVE THE REAL ROBOT at ${ip}?\n\n` +
            `It will first move from where it is now to the sim's home pose, then follow the same joints as the Sim ` +
            `at ${speed}% speed, stopping at each waypoint.\n\n` +
            `- Teach pendant in REMOTE mode, robot stopped, no alarms\n` +
            `- Work area clear; keep a hand on the E-stop\n` +
            (gripper
              ? `- The REAL gripper (USB) ${plan.touch ? 'closes before the arm moves (touch test)' : 'closes at the grasp and opens at the drop'}; if it is not activated, ` +
                `it first closes and opens once. Keep clear of the jaws.\n\n`
              : `- The gripper is NOT driven (only shown in the sim)\n\n`) +
            `Stop in the web app sends stop to the robot.`
    );
    if (!ok) return;
    this.busy = true;
    this.setStatus(fake ? 'Starting Real Sim with the fake robot...' : `Connecting to the robot at ${ip}...`);
    try {
      const runId = await startRealRun(this.runId, this.selected, { ip, src, speed, gripper });
      this.processRunning = true;
      this.live.start(runId, fake ? 'Real Sim: simulated robot (fake)' : `Real Sim: EC66 at ${ip} (live)`);
      this.watch(
        runId,
        (done, message) => {
          this.busy = false;
          this.live.finish(done, message);
          if (done) this.setStatus(message, 'ok');
          else this.fail(message);
        },
        (st) => st.state === 'running' && st.message && this.live.setMessage(st.message)
      );
    } catch (err: any) {
      this.fail(`Could not start Real Sim: ${err?.message || err}.`);
    }
  }

  private async goHome() {
    if (!this.runId || !this.doc || !this.realState().homeEnabled) return;
    const { ip, src, speed } = this.real;
    const fake = ip === 'fake';
    const ok = window.confirm(
      fake
        ? `Go Home with a SIMULATED robot (ip "fake") at ${speed}% speed?`
        : `MOVE THE REAL ROBOT at ${ip} to the home pose?\n\n` +
            `1. Raise the arm straight up in place (J2, J3, J4)\n` +
            `2. Turn the base (J1) and wrist while it is up\n` +
            `3. Lower into the home pose\n\n` +
            `Speed ${speed}%. The arm reaches about 1 m above its base when upright: check nothing is above it.\n` +
            `Work area clear, hand on the E-stop. Stop in the web app sends stop to the robot.`
    );
    if (!ok) return;
    this.busy = true;
    this.setStatus(fake ? 'Go Home with the fake robot...' : `Connecting to the robot at ${ip}...`);
    try {
      const runId = await startRealRun(this.runId, this.selected, { ip, src, speed, gripper: false }, 'home');
      this.processRunning = true;
      this.live.start(runId, fake ? 'Go Home: simulated robot (fake)' : `Go Home: EC66 at ${ip} (live)`);
      this.watch(
        runId,
        (done, message) => {
          this.busy = false;
          this.live.finish(done, message);
          if (done) this.setStatus(message, 'ok');
          else this.fail(message);
        },
        (st) => st.state === 'running' && st.message && this.live.setMessage(st.message)
      );
    } catch (err: any) {
      this.fail(`Could not start Go Home: ${err?.message || err}.`);
    }
  }

  private async stop() {
    try {
      await stopRun();
      this.setStatus('Stop sent.');
    } catch (err: any) {
      this.fail(`Could not send stop: ${err?.message || err}. Use the E-stop.`);
    }
  }

  // ---------------- status polling ----------------
  private watch(runId: string, onEnd: (ok: boolean, message: string, st?: SimStatus) => void, onUpdate?: (st: SimStatus) => void) {
    this.stopPolling();
    let lastMessage = '';
    let failures = 0;
    this.pollTimer = window.setInterval(async () => {
      let st;
      try {
        st = await fetchStatus(runId);
        failures = 0;
      } catch {
        // server restarting: keep polling, but not forever (an unreadable status would block every button)
        if (++failures * POLL_MS < 30000 || this.pollTimer === null) return;
        this.stopPolling();
        this.processRunning = false;
        this.live.finish(false, 'Lost contact with the cell server.');
        onEnd(false, `Could not read the job's status for 30 s (see var/runs/${runId}/log.txt). Is the cell server running?`);
        return;
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
      this.processRunning = false;
      onEnd(!!st.ok, st.message || 'The simulator ended without a message (see the run log in var/runs/<run>/log.txt).', st);
    }, POLL_MS);
  }

  private stopPolling() {
    if (this.pollTimer !== null) window.clearInterval(this.pollTimer);
    this.pollTimer = null;
  }
}
