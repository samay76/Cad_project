/**
 * Left-sidebar panel for the automatic pick & place: shows where the user is in
 * Load STEP / mesh -> Mark two points -> Plan -> Preview -> Sim -> Real Sim, and holds the actions.
 * Real Sim (the real EC66 follows the planned joints) is enabled only after the Sim of the same grasp ran to the end.
 * State lives in AutoPickController; this class only renders it.
 */
export interface AutoPickPanelEvents {
  onCalculate: () => void;
  onClearPoints: () => void;
  onExecute: () => void;
  onRealRun: () => void;
  onGoHome: () => void;
  onStop: () => void;
  onRobotSettings: (settings: { ip?: string; src?: string; speed?: number; gripper?: boolean }) => void;
  onPickOffset: (offset: { x?: number; y?: number; z?: number }) => void;
  onMode: (mode: 'grip' | 'touch') => void;
}

export interface AutoPickView {
  partName: string | null;
  mode: 'grip' | 'touch'; // grip: two points, one per finger; touch: the closed fingertips go to one point
  points: string[]; // marked points so far, at most two
  pointColors: string[];
  gripHint: { ok: boolean; text: string } | null; // what the two points mean for the gripper
  busy: boolean; // planning or a simulator run in progress
  playing: boolean; // viewport playback running
  status: { text: string; kind: 'info' | 'ok' | 'error' } | null;
  grasps: { title: string; detail: string }[];
  selected: number;
  pickOffset: { x: number; y: number; z: number }; // mm, robot base frame, added to the grasp position
  offsetHint: string | null; // set when the offset changed after planning
  real: {
    ip: string;
    src: string;
    speed: number;
    gripper: boolean;
    enabled: boolean; // Sim of this grasp has been checked and the plan is for the EC66
    homeEnabled: boolean; // the plan is for the EC66 (Go Home needs no Sim check: it only goes to the home pose)
    hint: string; // why it is disabled, or what it will do
    running: boolean; // a simulator or real-robot process is running and can be stopped
  };
}

const escapeHtml = (s: string) =>
  s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

export class AutoPickPanel {
  private events: AutoPickPanelEvents | null = null;

  constructor(private container: HTMLElement) {}

  public setEvents(events: AutoPickPanelEvents) {
    this.events = events;
  }

  public render(view: AutoPickView) {
    const step = (done: boolean, mark: string, text: string, color?: string) => `
      <div class="auto-pick-step ${done ? 'is-done' : ''}">
        <span class="auto-pick-step-mark" ${done && color ? `style="background:${color}"` : ''}>${mark}</span>
        <span class="auto-pick-step-text">${text}</span>
      </div>`;
    const touch = view.mode === 'touch';
    const pointStep = (i: number) => {
      const text = view.points[i];
      if (touch) {
        return step(!!text, '1', text ? `Point: ${escapeHtml(text)}` : 'Click the point the fingertips should go to (e.g. a number)', view.pointColors[0]);
      }
      const hint = view.points.length === i ? `Click the part where finger ${i + 1} should go` : `Finger ${i + 1}`;
      return step(!!text, String(i + 1), text ? `Finger ${i + 1}: ${escapeHtml(text)}` : hint, view.pointColors[i]);
    };
    const planned = view.grasps.length > 0;
    const canCalculate = !!view.partName && view.points.length === (touch ? 1 : 2) && !view.busy;

    this.container.innerHTML = `
      <div class="sidebar-block auto-pick-block">
        <div class="block-header"><span class="block-title">AUTO PICK &amp; PLACE</span></div>
        ${step(!!view.partName, view.partName ? '✓' : '·', view.partName ? escapeHtml(view.partName) : 'Load a CAD file (Load CAD)')}
        <div class="segmented-control auto-pick-mode" role="group" aria-label="What the robot does at the marked points">
          <button class="seg-btn ${touch ? '' : 'active'}" data-mode="grip" ${view.busy ? 'disabled' : ''} title="Mark two points: the gripper closes on them (one finger on each)">Grip (2 points)</button>
          <button class="seg-btn ${touch ? 'active' : ''}" data-mode="touch" ${view.busy ? 'disabled' : ''} title="Mark one point: the closed fingertips go to it and stop just before it (touch test)">Touch (1 point)</button>
        </div>
        ${view.partName ? (touch ? pointStep(0) : pointStep(0) + pointStep(1)) : ''}
        ${view.gripHint ? `<div id="auto-pick-grip-hint" class="auto-pick-hint ${view.gripHint.ok ? '' : 'is-bad'}">${escapeHtml(view.gripHint.text)}</div>` : ''}
        <div class="auto-pick-real-title">PICK OFFSET (mm, robot base X / Y / Z)</div>
        <div class="auto-pick-real-fields">
          ${(['x', 'y', 'z'] as const)
            .map(
              (k) =>
                `<label>${k.toUpperCase()}<input id="auto-pick-offset-${k}" type="number" step="0.5" min="-50" max="50" value="${view.pickOffset[k]}" ${view.busy ? 'disabled' : ''} title="Moves the gripper's pick position by this much (mm) in the robot base frame, in the Sim and on the real robot. Plan again after changing it."></label>`
            )
            .join('')}
        </div>
        ${view.offsetHint ? `<div class="auto-pick-hint is-bad">${escapeHtml(view.offsetHint)}</div>` : ''}
        <div class="auto-pick-actions">
          <button id="auto-pick-calculate" class="btn btn-primary btn-sm" ${canCalculate ? '' : 'disabled'} title="${touch ? 'Plan the motion that brings the closed fingertips to the marked point' : 'Plan the motion that puts one finger on each marked point'}">${touch ? 'Plan to Point' : 'Plan to Points'}</button>
          <button id="auto-pick-clear" class="btn btn-secondary btn-sm" ${view.points.length && !view.busy ? '' : 'disabled'} title="Remove the marked points">Clear Points</button>
        </div>
        ${
          planned
            ? `<div class="auto-pick-actions">
                <button id="auto-pick-execute" class="btn btn-primary auto-pick-sim" ${view.busy || view.playing ? 'disabled' : ''} title="Run the motion with full physics in the simulation, shown live in the 3D view. Let it finish to enable Real Sim.">Simulation</button>
              </div>
              <div class="auto-pick-real">
                <div class="auto-pick-real-title">REAL ROBOT (Elite EC66)</div>
                <div class="auto-pick-real-fields">
                  <label>IP<input id="auto-pick-real-ip" type="text" value="${escapeHtml(view.real.ip)}" placeholder="192.168.1.200 or fake" ${view.busy ? 'disabled' : ''}></label>
                  <label>From<input id="auto-pick-real-src" type="text" value="${escapeHtml(view.real.src)}" placeholder="optional" ${view.busy ? 'disabled' : ''}></label>
                  <label>Speed %<input id="auto-pick-real-speed" type="number" min="1" max="10" step="1" value="${view.real.speed}" ${view.busy ? 'disabled' : ''}></label>
                </div>
                <label class="auto-pick-real-check"><input id="auto-pick-real-gripper" type="checkbox" ${view.real.gripper ? 'checked' : ''} ${view.busy ? 'disabled' : ''}> Real gripper (Robotiq 2F-85 on USB)</label>
                <div class="auto-pick-hint">${escapeHtml(view.real.hint)}</div>
                <div class="auto-pick-actions">
                  <button id="auto-pick-go-home" class="btn btn-secondary btn-sm auto-pick-wide" ${view.real.homeEnabled && !view.busy && !view.playing ? '' : 'disabled'} title="Bring the real robot from wherever it is to this plan's home pose: arm straight up, turn, lower. Checked against the table first.">Go Home (real robot)</button>
                  <button id="auto-pick-real-run" class="btn btn-primary btn-sm" ${view.real.enabled && !view.busy && !view.playing ? '' : 'disabled'} title="Move the real robot through the same joints as the Sim, with the MuJoCo window following it live">Real Sim</button>
                  <button id="auto-pick-stop" class="btn btn-secondary btn-sm auto-pick-stop" ${view.real.running ? '' : 'disabled'} title="Stop the simulator; during Real Sim this sends stop to the robot. Keep a hand on the E-stop anyway.">Stop</button>
                </div>
              </div>`
            : ''
        }
      </div>
    `;

    const on = (id: string, handler?: () => void) =>
      this.container.querySelector(`#${id}`)?.addEventListener('click', () => handler?.());
    on('auto-pick-calculate', this.events?.onCalculate);
    on('auto-pick-clear', this.events?.onClearPoints);
    on('auto-pick-execute', this.events?.onExecute);
    on('auto-pick-real-run', this.events?.onRealRun);
    on('auto-pick-go-home', this.events?.onGoHome);
    on('auto-pick-stop', this.events?.onStop);
    // Saved on every keystroke, so a re-render of the panel never loses what was typed.
    const field = (id: string, apply: (v: string) => void) =>
      this.container.querySelector<HTMLInputElement>(`#${id}`)?.addEventListener('input', (e) => apply((e.target as HTMLInputElement).value.trim()));
    field('auto-pick-real-ip', (ip) => this.events?.onRobotSettings({ ip }));
    field('auto-pick-real-src', (src) => this.events?.onRobotSettings({ src }));
    field('auto-pick-real-speed', (v) => this.events?.onRobotSettings({ speed: Number(v) }));
    for (const k of ['x', 'y', 'z'] as const) {
      field(`auto-pick-offset-${k}`, (v) => this.events?.onPickOffset({ [k]: Number(v) || 0 }));
    }
    this.container.querySelector<HTMLInputElement>('#auto-pick-real-gripper')?.addEventListener('change', (e) =>
      this.events?.onRobotSettings({ gripper: (e.target as HTMLInputElement).checked })
    );
    this.container.querySelectorAll<HTMLButtonElement>('.auto-pick-mode .seg-btn').forEach((btn) => {
      btn.addEventListener('click', () => this.events?.onMode(btn.dataset.mode as 'grip' | 'touch'));
    });
  }
}
