/**
 * The operator's step screens: the stepper row, the panel for the current step (one primary button each), the
 * chip at the top left of the 3D view, and the robot connection fields in Settings.
 * State of the job lives in AutoPickController; this class renders it and keeps only screen state (which step
 * is shown, the safety ticks, the speed choice, the fine-tune drawer).
 */
import { JOG_STEPS, SPEED_PRESETS, type JogStep, type SpeedPreset } from '../config';
import { friendly, progressLabel, type Job } from './friendly';
import { icon } from './icons';
import { STEPS, isTeachMode, canOpen, canStartRobot, clampStep, stepDone, type SafetyChecks, type StepFacts, type StepId } from './steps';

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
  onJog: (axis: 'x' | 'y' | 'z', mm: number) => void;
  onTurn: (axis: 'x' | 'y' | 'z', deg: number) => void;
  onTeachMode: (on: boolean) => void; // from the current step (isTeachMode)
}

/** File choosing is the layout's job (the file input lives there). */
export interface FileActions {
  pickFile: () => void;
  handleFile: (file: File) => void;
  /** Draws the fixture library into the Load part step (kept by the library, re-drawn on every render). */
  renderFixtures: (el: HTMLElement) => void;
}

export interface AutoPickView {
  partName: string | null;
  mode: 'grip' | 'touch';
  points: string[]; // marked points so far (raw coordinates: shown only in Fine-tune)
  pointColors: string[];
  gripHint: { ok: boolean; text: string } | null;
  gapMm: number | null; // distance between the two marked points
  busy: boolean;
  playing: boolean;
  status: { text: string; kind: 'info' | 'ok' | 'error' } | null;
  grasps: { title: string; detail: string }[];
  selected: number;
  pickOffset: { x: number; y: number; z: number };
  offsetHint: string | null;
  plan: { mode: 'outside' | 'inside'; tiltDeg: number; touch: boolean; durationS: number } | null;
  planError: string | null; // the last "Find best grip" failed with this message
  job: Job; // the job running now
  jobStartedAt: number | null; // ms (performance.now) when it started
  phase: string | null; // its current phase
  sim: 'none' | 'running' | 'ok' | 'failed';
  simMessage: string | null;
  robot: 'none' | 'running' | 'ok' | 'failed';
  robotMessage: string | null;
  real: {
    ip: string;
    src: string;
    speed: number;
    gripper: boolean;
    enabled: boolean;
    homeEnabled: boolean;
    hint: string;
    running: boolean;
  };
}

const escapeHtml = (s: string) =>
  s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

const clock = (s: number) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

export class AutoPickPanel {
  private events: AutoPickPanelEvents | null = null;
  private view: AutoPickView | null = null;
  private step: StepId = 1;
  private prev: StepFacts | null = null;
  private checks: SafetyChecks = { areaClear: false, partInPlace: false };
  private speed: SpeedPreset = 'slow';
  private fineTune = false;
  private jogStep: JogStep = 'fine';
  private notice: string | null = null;
  private noticeTimer: number | null = null;
  private chipTimer: number | null = null;
  private settingsBuilt = false;

  constructor(
    private container: HTMLElement,
    private mounts: { stepper: HTMLElement; chip: HTMLElement; settings: HTMLElement },
    private files: FileActions
  ) {}

  public setEvents(events: AutoPickPanelEvents) {
    this.events = events;
    events.onTeachMode(isTeachMode(this.step));
  }

  public get currentStep(): StepId {
    return this.step;
  }

  private facts(v: AutoPickView): StepFacts {
    return {
      partLoaded: !!v.partName,
      planned: v.grasps.length > 0,
      simChecked: v.real.enabled,
      robotDone: v.robot === 'ok'
    };
  }

  public render(view: AutoPickView) {
    this.view = view;
    const f = this.facts(view);
    const p = this.prev;
    // Follow the job forward: a new part opens step 1, a found grip opens step 3.
    if (p && !p.planned && f.planned && this.step <= 2) this.goTo(3, false);
    else if (p && f.partLoaded && !p.partLoaded) this.goTo(1, false);
    this.prev = f;
    const clamped = clampStep(this.step, f);
    if (clamped !== this.step) this.goTo(clamped, false);

    this.renderStepper(f);
    this.renderPanel(view, f);
    this.renderChip();
    this.renderSettings(view);
  }

  /** Opens a step. Speed goes back to Slow and the safety ticks are cleared each time the run step opens. */
  private goTo(step: StepId, userClick = true) {
    if (this.view && userClick) {
      const ok = canOpen(step, this.facts(this.view));
      if (!ok.ok) return this.flash(ok.reason);
    }
    if (step === 4 && this.step !== 4) {
      this.checks = { areaClear: false, partInPlace: false };
      this.setSpeed('slow');
    }
    if (step !== 2) this.fineTune = false;
    this.step = step;
    this.events?.onTeachMode(isTeachMode(step)); // leaving Teach grip turns teach mode off
    if (userClick && this.view) this.render(this.view);
  }

  private setSpeed(preset: SpeedPreset) {
    this.speed = preset;
    this.events?.onRobotSettings({ speed: SPEED_PRESETS[preset] });
  }

  private flash(text: string) {
    this.notice = text;
    if (this.noticeTimer !== null) window.clearTimeout(this.noticeTimer);
    this.noticeTimer = window.setTimeout(() => {
      this.notice = null;
      if (this.view) this.render(this.view);
    }, 4000);
    if (this.view) this.render(this.view);
  }

  // ---------------- stepper ----------------
  private renderStepper(f: StepFacts) {
    this.mounts.stepper.innerHTML = `
      <ol class="op-steps">
        ${STEPS.map((s) => {
          const done = stepDone(s.id, f);
          const current = s.id === this.step;
          const open = canOpen(s.id, f);
          const cls = current ? 'is-current' : done ? 'is-done' : open.ok ? 'is-open' : 'is-locked';
          return `<li><button class="op-step ${cls}" data-step="${s.id}" ${current ? 'aria-current="step"' : ''}
              title="${open.ok ? escapeHtml(s.label) : escapeHtml(open.reason)}">
            <span class="op-step-num">${done && !current ? icon('check', 18) : s.id}</span>
            <span class="op-step-label">${escapeHtml(s.label)}</span>
          </button></li>`;
        }).join('')}
      </ol>
      ${this.notice ? `<div class="op-notice" role="alert">${icon('info', 16)}<span>${escapeHtml(this.notice)}</span></div>` : ''}`;
    this.mounts.stepper.querySelectorAll<HTMLButtonElement>('.op-step').forEach((b) =>
      b.addEventListener('click', () => this.goTo(Number(b.dataset.step) as StepId))
    );
  }

  // ---------------- chip on the 3D view ----------------
  private renderChip() {
    const v = this.view!;
    const job = v.real.running || v.busy ? v.job : null;
    const elapsed = v.jobStartedAt !== null ? (performance.now() - v.jobStartedAt) / 1000 : 0;
    const total = v.plan?.durationS ?? null;
    const timed = (label: string) => `${label} · ${clock(elapsed)}${total ? ` / ${clock(total)}` : ''}`;
    let text: string;
    let live = !!job;
    switch (job) {
      case 'plan':
        text = `Finding a grip · ${clock(elapsed)}`;
        break;
      case 'approach':
        text = 'Moving to the part';
        break;
      case 'sim':
        text = timed('Simulation');
        break;
      case 'real':
        text = timed('Robot running');
        break;
      case 'home':
        text = `Going home · ${clock(elapsed)}`;
        break;
      default:
        live = false;
        text = !v.partName ? 'No part loaded' : this.step === 2 ? 'Teaching' : this.step === 3 ? 'Simulation' : this.step === 4 ? 'Robot' : 'Part loaded';
    }
    this.mounts.chip.innerHTML = `${live ? '<span class="op-chip-dot"></span>' : ''}<span>${escapeHtml(text)}</span>`;
    this.mounts.chip.classList.toggle('is-live', live);
    if (live && this.chipTimer === null) this.chipTimer = window.setInterval(() => this.view && this.renderChip(), 500);
    if (!live && this.chipTimer !== null) {
      window.clearInterval(this.chipTimer);
      this.chipTimer = null;
    }
  }

  // ---------------- step panel ----------------
  private renderPanel(v: AutoPickView, f: StepFacts) {
    const body = [this.step1, this.step2, this.step3, this.step4][this.step - 1].call(this, v, f);
    const titles = ['Load your part', 'Show where to grip', 'Check the simulation', 'Run on the real robot'];
    const lines = [
      'Choose a fixture, then the part the robot should pick.',
      v.mode === 'touch'
        ? 'Click one spot on the part where the closed fingertips should go.'
        : 'Click two spots on the part where the fingers should touch.',
      'Watch the robot pick the part in the simulation before it runs for real.',
      'The robot will do exactly what you saw in the simulation.'
    ];
    this.container.innerHTML = `
      <div class="op-card op-step-card">
        <div class="op-step-of">Step ${this.step} of 4</div>
        <h2 class="op-title">${titles[this.step - 1]}</h2>
        <p class="op-lead">${lines[this.step - 1]}</p>
        ${body}
      </div>`;
    this.bindPanel();
  }

  private resultCard(kind: 'ok' | 'error' | 'info' | 'warn', title: string, line: string) {
    const ic = { ok: 'checkCircle', error: 'error', info: 'info', warn: 'warn' }[kind];
    return `<div class="op-result is-${kind}" role="status">
      <span class="op-result-icon">${icon(ic, 22)}</span>
      <div><div class="op-result-title">${escapeHtml(title)}</div>${line ? `<div class="op-result-line">${escapeHtml(line)}</div>` : ''}</div>
    </div>`;
  }

  private step1(v: AutoPickView) {
    return `
      <div id="op-fixtures" class="op-fixtures"></div>
      ${v.partName ? this.resultCard('ok', `${v.partName} loaded`, '') : ''}
      <div class="op-actions">
        <button class="btn btn-primary op-btn op-primary" id="op-next-1" ${v.partName ? '' : 'disabled'} title="${v.partName ? '' : 'Choose a part from a fixture first.'}">Next: teach grip</button>
      </div>`;
  }

  private step2(v: AutoPickView) {
    const touch = v.mode === 'touch';
    const need = touch ? 1 : 2;
    const row = (i: number) => {
      const placed = !!v.points[i];
      const name = touch ? 'Fingertip spot' : `Finger ${i + 1}`;
      return `<li class="op-point ${placed ? 'is-placed' : ''}">
        <span class="op-badge" style="background:${v.pointColors[i]}">${i + 1}</span>
        <span class="op-point-text">${name} ${placed ? 'placed' : v.points.length === i ? '- click the part' : 'not placed'}</span>
        <span class="op-point-check">${placed ? icon('check', 18) : ''}</span>
      </li>`;
    };
    const ready = !!v.partName && v.points.length === need && !v.busy;
    const planning = v.busy && v.job === 'plan';
    const hintBad = v.gripHint && !v.gripHint.ok;
    return `
      <div class="segmented-control op-seg auto-pick-mode" role="group" aria-label="Grip or touch">
        <button class="seg-btn ${touch ? '' : 'active'}" data-mode="grip" ${v.busy ? 'disabled' : ''}>Grip (2 points)</button>
        <button class="seg-btn ${touch ? 'active' : ''}" data-mode="touch" ${v.busy ? 'disabled' : ''}>Touch (1 point)</button>
      </div>
      <ul class="op-points">${row(0)}${touch ? '' : row(1)}</ul>
      ${!touch && v.gapMm !== null ? `<div class="op-gap">Gap between fingers: <b>${v.gapMm.toFixed(0)} mm</b></div>` : ''}
      ${hintBad ? this.resultCard('warn', 'These spots will not work', friendly(v.gripHint!.text, 'error').text) : ''}
      ${planning ? this.resultCard('info', 'Finding the best grip…', v.status ? friendly(v.status.text).text : '') : ''}
      ${!planning && v.planError ? this.resultCard('error', "Robot can't reach this grip", 'Try moving the grip points or the part position.') : ''}
      ${v.offsetHint ? this.resultCard('warn', 'Position changed', 'Press "Find best grip" to use the new position.') : ''}
      <button class="btn btn-secondary op-btn op-wide" id="op-finetune" aria-expanded="${this.fineTune}">${icon('sliders', 18)}<span>Fine-tune position</span></button>
      ${this.fineTune ? this.fineTuneDrawer(v) : ''}
      <div class="op-actions">
        <button id="auto-pick-calculate" class="btn btn-primary op-btn op-primary" ${ready ? '' : 'disabled'}
          title="${ready ? '' : v.busy ? 'Wait until the current job has finished.' : touch ? 'Click one spot on the part first.' : 'Click two spots on the part first.'}">
          ${planning ? 'Finding grip…' : 'Find best grip'}</button>
        <button id="auto-pick-clear" class="btn btn-secondary op-btn" ${v.points.length && !v.busy ? '' : 'disabled'}
          title="${v.points.length ? 'Remove the points and click them again.' : 'No points placed yet.'}">${icon('resetView', 16)}<span>Reset points</span></button>
      </div>`;
  }

  private fineTuneDrawer(v: AutoPickView) {
    const dis = v.busy ? 'disabled' : '';
    const pad = (axis: string, dir: number, ic: string, label: string) =>
      `<button class="op-pad-btn" data-jog="${axis}" data-dir="${dir}" ${dis} title="${label}" aria-label="${label}">${icon(ic, 20)}</button>`;
    const turn = (axis: string, dir: number, ic: string, label: string) =>
      `<button class="op-pad-btn op-turn" data-turn="${axis}" data-dir="${dir}" ${dis} title="${label}" aria-label="${label}">${icon(ic, 18)}<span>${label}</span></button>`;
    return `
      <div class="op-drawer-inline">
        <div class="op-sub">Move the gripper</div>
        <div class="segmented-control op-seg" role="group" aria-label="Step size">
          ${(Object.keys(JOG_STEPS) as JogStep[])
            .map((k) => `<button class="seg-btn ${this.jogStep === k ? 'active' : ''}" data-jogstep="${k}" title="${JOG_STEPS[k].mm} mm / ${JOG_STEPS[k].deg}°">${k[0].toUpperCase() + k.slice(1)}</button>`)
            .join('')}
        </div>
        <div class="op-pad">
          <div class="op-pad-grid">
            <span></span>${pad('y', 1, 'up', 'Forward')}<span></span>
            ${pad('x', -1, 'left', 'Left')}<span class="op-pad-mid">${icon('box', 16)}</span>${pad('x', 1, 'right', 'Right')}
            <span></span>${pad('y', -1, 'down', 'Back')}<span></span>
          </div>
          <div class="op-pad-col">
            ${pad('z', 1, 'arrowUp', 'Up')}
            ${pad('z', -1, 'arrowDown', 'Down')}
          </div>
        </div>
        <div class="op-turns">
          ${turn('z', 1, 'rotL', 'Turn left')}${turn('z', -1, 'rotR', 'Turn right')}
          ${turn('x', 1, 'rotL', 'Tilt forward')}${turn('x', -1, 'rotR', 'Tilt back')}
          ${turn('y', 1, 'rotL', 'Tilt left')}${turn('y', -1, 'rotR', 'Tilt right')}
        </div>
        <div class="op-sub">Pick offset (mm)</div>
        <div class="op-fields">
          ${(['x', 'y', 'z'] as const)
            .map((k) => `<label>${k.toUpperCase()}<input id="auto-pick-offset-${k}" type="number" step="0.5" min="-50" max="50" value="${v.pickOffset[k]}" ${dis}></label>`)
            .join('')}
        </div>
        ${v.points.length ? `<div class="op-sub">Grip points (mm)</div><ul class="op-coords">${v.points.map((p, i) => `<li><span class="op-badge op-badge-sm" style="background:${v.pointColors[i]}">${i + 1}</span>${escapeHtml(p)}</li>`).join('')}</ul>` : ''}
        ${v.gripHint && v.gripHint.ok ? `<div class="op-small">${escapeHtml(v.gripHint.text)}</div>` : ''}
      </div>`;
  }

  private planLine(v: AutoPickView) {
    const p = v.plan;
    if (!p) return '';
    const dir = p.tiltDeg < 5 ? 'straight down' : `tilted ${Math.round(p.tiltDeg)}°`;
    if (p.touch) return `Fingertips go to the point · ${dir} · no collisions`;
    return `Fingers grip from ${p.mode} · ${dir} · no collisions`;
  }

  private step3(v: AutoPickView, f: StepFacts) {
    const running = v.sim === 'running' && v.busy;
    let card: string;
    if (running) card = this.resultCard('info', 'Simulation running…', progressLabel(v.phase ?? undefined, v.status?.text) ?? 'Watch the robot in the 3D view.');
    else if (v.sim === 'failed') card = this.resultCard('error', 'The simulation had a problem', friendly(v.simMessage || '', 'error').text);
    else if (f.simChecked) card = this.resultCard('ok', 'Grip looks good', this.planLine(v) + '. Simulation finished.');
    else card = this.resultCard('ok', 'Grip looks good', this.planLine(v));
    const canSim = !v.busy && !v.playing;
    const simDone = f.simChecked;
    return `
      ${card}
      <div class="op-actions">
        ${
          simDone
            ? `<button id="auto-pick-execute" class="btn btn-secondary op-btn" ${canSim ? '' : 'disabled'}>${icon('play', 16)}<span>Play again</span></button>
               <button class="btn btn-primary op-btn op-primary" id="op-next-3">Next: run on robot</button>`
            : `<button id="auto-pick-execute" class="btn btn-primary op-btn op-primary" ${canSim ? '' : 'disabled'} title="${canSim ? '' : 'Wait until the current job has finished.'}">${icon('play', 16)}<span>${running ? 'Simulation running…' : 'Run simulation'}</span></button>
               <button class="btn btn-secondary op-btn" id="op-next-3" disabled title="Run the simulation to the end first.">${icon('lock', 16)}<span>Next: run on robot</span></button>`
        }
        <button class="btn btn-text op-btn" id="op-reteach">${icon('back', 16)}<span>Re-teach grip points</span></button>
      </div>`;
  }

  private step4(v: AutoPickView, f: StepFacts) {
    const running = v.robot === 'running' && v.real.running;
    const gate = canStartRobot(f, this.checks, v.busy || v.playing);
    let card = '';
    if (running) {
      card = this.resultCard('info', v.job === 'home' ? 'Going home…' : progressLabel(v.phase ?? undefined, v.status?.text) ?? 'Starting…', 'Press Stop robot at the top right to stop at once.');
    } else if (v.robot === 'ok') {
      card = this.resultCard('ok', 'Done', v.job === 'home' ? 'The robot is at home.' : 'The robot finished the pick and place.');
    } else if (v.robot === 'failed') {
      card = this.resultCard('error', 'The robot stopped', friendly(v.robotMessage || '', 'error').text);
    }
    const dis = running || v.busy ? 'disabled' : '';
    const done = !running && v.robot === 'ok' && v.job !== 'home'; // a pick and place just finished
    return `
      <div class="op-warning" role="note">${icon('warn', 22)}<span>Make sure nobody is inside the robot area.</span></div>
      <label class="op-check"><input type="checkbox" id="op-check-area" ${this.checks.areaClear ? 'checked' : ''} ${dis}><span>Area is clear</span></label>
      <label class="op-check"><input type="checkbox" id="op-check-part" ${this.checks.partInPlace ? 'checked' : ''} ${dis}><span>Part is in place</span></label>
      <div class="op-sub">Speed</div>
      <div class="segmented-control op-seg" role="group" aria-label="Speed">
        ${(Object.keys(SPEED_PRESETS) as SpeedPreset[])
          .map((k) => `<button class="seg-btn ${this.speed === k ? 'active' : ''}" data-speed="${k}" ${dis} title="${SPEED_PRESETS[k]} % speed">${k[0].toUpperCase() + k.slice(1)}</button>`)
          .join('')}
      </div>
      ${card}
      <div class="op-actions">
        ${done ? `<button id="op-teach-next" class="btn btn-primary op-btn op-primary" ${dis}>${icon('back', 16)}<span>Teach next part</span></button>` : ''}
        <span class="op-gate" title="${escapeHtml(gate.reason)}">
          <button id="auto-pick-real-run" class="btn ${done ? 'btn-secondary' : 'btn-primary op-primary'} op-btn" ${gate.ok ? '' : 'disabled'} aria-describedby="op-gate-reason">
            ${gate.ok ? icon('play', 16) : icon('lock', 16)}<span>${running ? 'Robot running…' : done ? 'Run again' : 'Start robot'}</span></button>
        </span>
        ${gate.ok ? '' : `<div class="op-small" id="op-gate-reason">${escapeHtml(gate.reason)}</div>`}
        <button id="auto-pick-go-home" class="btn btn-text op-btn" ${v.real.homeEnabled && !v.busy && !v.playing ? '' : 'disabled'}>${icon('home', 16)}<span>Send robot home</span></button>
        <button id="op-back-teach" class="btn btn-text op-btn" ${dis}>${icon('back', 16)}<span>Back to teaching</span></button>
      </div>`;
  }

  private bindPanel() {
    const c = this.container;
    const on = (id: string, fn: () => void) => c.querySelector(`#${id}`)?.addEventListener('click', fn);
    const ev = this.events;
    // step 1
    const lib = c.querySelector<HTMLElement>('#op-fixtures');
    if (lib) this.files.renderFixtures(lib);
    on('op-next-1', () => this.goTo(2));
    // step 2
    c.querySelectorAll<HTMLButtonElement>('.auto-pick-mode .seg-btn').forEach((b) =>
      b.addEventListener('click', () => ev?.onMode(b.dataset.mode as 'grip' | 'touch'))
    );
    on('op-finetune', () => {
      this.fineTune = !this.fineTune;
      this.render(this.view!);
    });
    c.querySelectorAll<HTMLButtonElement>('[data-jogstep]').forEach((b) =>
      b.addEventListener('click', () => {
        this.jogStep = b.dataset.jogstep as JogStep;
        this.render(this.view!);
      })
    );
    c.querySelectorAll<HTMLButtonElement>('[data-jog]').forEach((b) =>
      b.addEventListener('click', () => ev?.onJog(b.dataset.jog as 'x' | 'y' | 'z', Number(b.dataset.dir) * JOG_STEPS[this.jogStep].mm))
    );
    c.querySelectorAll<HTMLButtonElement>('[data-turn]').forEach((b) =>
      b.addEventListener('click', () => ev?.onTurn(b.dataset.turn as 'x' | 'y' | 'z', Number(b.dataset.dir) * JOG_STEPS[this.jogStep].deg))
    );
    for (const k of ['x', 'y', 'z'] as const) {
      c.querySelector<HTMLInputElement>(`#auto-pick-offset-${k}`)?.addEventListener('input', (e) =>
        ev?.onPickOffset({ [k]: Number((e.target as HTMLInputElement).value) || 0 })
      );
    }
    on('auto-pick-clear', () => {
      if (window.confirm('Reset the grip points? You will click them again on the part.')) ev?.onClearPoints();
    });
    on('auto-pick-calculate', () => ev?.onCalculate());
    // step 3
    on('auto-pick-execute', () => ev?.onExecute());
    on('op-next-3', () => this.goTo(4));
    on('op-reteach', () => this.goTo(2));
    // step 4: after a run, back to teaching (this part's grip, or the next part from the fixture library)
    on('op-back-teach', () => this.goTo(2));
    on('op-teach-next', () => this.goTo(1));
    // step 4
    const tick = (id: string, key: keyof SafetyChecks) =>
      c.querySelector<HTMLInputElement>(`#${id}`)?.addEventListener('change', (e) => {
        this.checks[key] = (e.target as HTMLInputElement).checked;
        this.render(this.view!);
      });
    tick('op-check-area', 'areaClear');
    tick('op-check-part', 'partInPlace');
    c.querySelectorAll<HTMLButtonElement>('[data-speed]').forEach((b) =>
      b.addEventListener('click', () => {
        this.setSpeed(b.dataset.speed as SpeedPreset);
        this.render(this.view!);
      })
    );
    on('auto-pick-real-run', () => {
      // Checked again here: the button state could be stale if the view changed under the pointer.
      const gate = canStartRobot(this.facts(this.view!), this.checks, this.view!.busy || this.view!.playing);
      if (!gate.ok) return this.flash(gate.reason);
      // The speed the operator chose on this screen, sent right before the start.
      this.events?.onRobotSettings({ speed: SPEED_PRESETS[this.speed] });
      ev?.onRealRun();
    });
    on('auto-pick-go-home', () => ev?.onGoHome());
  }

  // ---------------- Settings > Robot connection ----------------
  private renderSettings(v: AutoPickView) {
    const m = this.mounts.settings;
    if (!this.settingsBuilt) {
      m.innerHTML = `
        <div class="op-section-body op-form">
          <label>Robot address (IP)<input id="auto-pick-real-ip" type="text" placeholder="192.168.1.200 or fake"></label>
          <label>Connect from (this PC's IP, optional)<input id="auto-pick-real-src" type="text" placeholder="optional"></label>
          <label class="op-row"><span>Real gripper (Robotiq 2F-85 on USB)</span>
            <span class="op-switch"><input id="auto-pick-real-gripper" type="checkbox"><span></span></span></label>
          <div class="op-small">Speed is chosen on the Run on robot step (Slow ${SPEED_PRESETS.slow} %, Normal ${SPEED_PRESETS.normal} %, Fast ${SPEED_PRESETS.fast} %).</div>
        </div>`;
      const field = (id: string, apply: (v: string) => void) =>
        m.querySelector<HTMLInputElement>(`#${id}`)!.addEventListener('input', (e) => apply((e.target as HTMLInputElement).value.trim()));
      field('auto-pick-real-ip', (ip) => this.events?.onRobotSettings({ ip }));
      field('auto-pick-real-src', (src) => this.events?.onRobotSettings({ src }));
      m.querySelector<HTMLInputElement>('#auto-pick-real-gripper')!.addEventListener('change', (e) =>
        this.events?.onRobotSettings({ gripper: (e.target as HTMLInputElement).checked })
      );
      this.settingsBuilt = true;
    }
    const set = (id: string, value: string) => {
      const el = m.querySelector<HTMLInputElement>(`#${id}`)!;
      if (document.activeElement !== el && el.value !== value) el.value = value;
      el.disabled = v.busy;
    };
    set('auto-pick-real-ip', v.real.ip);
    set('auto-pick-real-src', v.real.src);
    const g = m.querySelector<HTMLInputElement>('#auto-pick-real-gripper')!;
    g.checked = v.real.gripper;
    g.disabled = v.busy;
  }
}
