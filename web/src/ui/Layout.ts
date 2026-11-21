/**
 * The operator layout:
 *   header (name, job, robot / gripper status, settings, Stop robot - always there, always enabled)
 *   stepper (Load part -> Teach grip -> Simulate -> Run on robot)
 *   3D view (left) + the current step's panel (right)
 *   "What happened" (bottom)
 *   Settings drawer: robot connection, gripper, part pose, CAD properties, jogging, engineer log.
 */
import { GRIPPERS, type GripperId } from '../config';
import type { Pill, Pills } from './friendly';
import { icon } from './icons';

export interface LayoutCallbacks {
  onFileSelected: (file: File) => void;
  onLoadGripperClick: () => void;
  onTransformModeToggle: (mode: 'translate' | 'rotate') => void;
  onTargetToggle?: (target: 'part' | 'gripper') => void;
  onStop: () => void;
  onResetView: () => void;
  onTopView: () => void;
}

const THEME_KEY = 'ui.theme';

const escapeHtml = (s: string) =>
  s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

export class AppLayout {
  private root: HTMLElement;
  private callbacks: LayoutCallbacks;

  // Mount points
  public topBarContainer!: HTMLElement;
  public stepperContainer!: HTMLElement;
  public autoPickContainer!: HTMLElement; // the step panel
  public activityContainer!: HTMLElement;
  public viewChipContainer!: HTMLElement;
  public settingsRobotContainer!: HTMLElement;
  public partsListContainer!: HTMLElement;
  public viewportContainer!: HTMLElement;
  public gripperPanelContainer!: HTMLElement;
  public partPanelContainer!: HTMLElement;
  public propertiesPanelContainer!: HTMLElement;
  public logContainer!: HTMLElement;

  private fileInput!: HTMLInputElement;
  private jobText!: HTMLElement;
  private gripperStatusBadge!: HTMLElement;
  private dragOverlay!: HTMLElement;
  private loadingOverlay!: HTMLElement;
  private loadingMessage!: HTMLElement;
  private errorToast!: HTMLElement;
  private errorMessage!: HTMLElement;
  private errorTitle!: HTMLElement;
  private drawer!: HTMLElement;
  private scrim!: HTMLElement;

  private btnTbEditGripper!: HTMLButtonElement;
  private btnTbEditPart!: HTMLButtonElement;
  private btnTbTrans!: HTMLButtonElement;
  private btnTbRot!: HTMLButtonElement;

  private partName: string | null = null;
  private selection: { fixture: string; part: string } | null = null;
  private station = 1;

  constructor(root: HTMLElement, callbacks: LayoutCallbacks) {
    this.root = root;
    this.callbacks = callbacks;
    this.applyTheme(this.savedTheme());
    this.initDOM();
    this.bindEvents();
  }

  private initDOM() {
    this.root.innerHTML = `
      <div class="op-layout">
        <header class="op-header" id="top-bar-container">
          <div class="op-brand">
            <div class="op-brand-icon">${icon('box', 22)}</div>
            <div class="op-brand-text">
              <span class="op-brand-title">Pick-point</span>
              <span class="op-brand-sub" id="op-job">Job: no part loaded · Station 1</span>
            </div>
          </div>
          <div class="op-header-right">
            <div class="op-pill" id="pill-robot" data-state="idle" role="status" aria-live="polite">
              <span class="op-pill-icon"></span><span class="op-pill-text">Robot not checked yet</span>
            </div>
            <div class="op-pill" id="pill-gripper" data-state="idle" role="status" aria-live="polite">
              <span class="op-pill-icon"></span><span class="op-pill-text">Gripper not checked yet</span>
            </div>
            <button class="op-icon-btn" id="op-settings-btn" title="Settings" aria-label="Settings">${icon('settings', 22)}</button>
            <button class="op-stop" id="op-stop" title="Stop the robot now. Keep a hand on the emergency stop as well.">
              ${icon('stop', 22)}<span>Stop robot</span>
            </button>
          </div>
        </header>

        <nav class="op-stepper" id="op-stepper" aria-label="Steps"></nav>

        <main class="op-main">
          <section class="op-viewport viewport-section" id="viewport-container">
            <div class="op-view-chip" id="op-view-chip"></div>
            <div class="op-view-tools">
              <button class="op-tool" id="op-help" aria-label="How to move the view">${icon('help', 20)}
                <span class="op-tooltip" role="tooltip">
                  <b>Left drag</b> rotate · <b>Right drag</b> pan · <b>Scroll</b> zoom<br>
                  <b>Click the part</b> to place a grip point
                </span>
              </button>
              <button class="op-tool" id="op-reset-view" title="Reset view" aria-label="Reset view">${icon('resetView', 20)}</button>
              <button class="op-tool" id="op-top-view" title="Top view" aria-label="Top view">${icon('topView', 20)}</button>
            </div>
            <div class="drag-drop-overlay" id="drag-overlay">
              <div class="drag-drop-content">
                ${icon('upload', 48)}
                <h3>Drop the STEP file here</h3>
                <p>.step, .stp, .stl or .obj</p>
              </div>
            </div>
            <div class="loading-overlay" id="loading-overlay">
              <div class="loading-spinner"></div>
              <p class="loading-message" id="loading-message">Opening the part…</p>
            </div>
          </section>

          <aside class="op-panel" id="auto-pick-mount"></aside>
        </main>

        <section class="op-activity" id="op-activity" aria-label="Status"></section>

        <div class="op-scrim" id="op-scrim"></div>
        <aside class="op-drawer" id="op-drawer" aria-label="Settings" aria-hidden="true">
          <div class="op-drawer-head">
            <span class="op-drawer-title">Settings</span>
            <button class="op-icon-btn" id="op-drawer-close" aria-label="Close settings">${icon('close', 22)}</button>
          </div>
          <div class="op-drawer-body">
            <details class="op-section" open>
              <summary>Robot connection</summary>
              <div id="op-settings-robot"></div>
            </details>
            <details class="op-section">
              <summary>Gripper</summary>
              <div class="op-section-body">
                <div class="op-row">
                  <div class="gripper-badge" id="gripper-status-badge">
                    <span class="gripper-dot"></span><span class="gripper-text">Robotiq 2F-85</span>
                  </div>
                  <button id="btn-load-gripper" class="btn btn-secondary op-btn-sm">Load gripper</button>
                </div>
                <div id="gripper-panel-mount"></div>
              </div>
            </details>
            <details class="op-section">
              <summary>Part pose</summary>
              <div class="op-section-body">
                <div class="viewport-toolbar op-toolbar" id="viewport-toolbar">
                  <div class="toolbar-target-group">
                    <button id="tb-btn-edit-gripper" class="tb-btn tb-btn-target active" title="Edit gripper pose (G)">Gripper</button>
                    <button id="tb-btn-edit-part" class="tb-btn tb-btn-target" title="Edit part pose (P)">Part</button>
                  </div>
                  <div class="toolbar-group">
                    <button id="tb-btn-trans" class="tb-btn active" title="Move (T)">Move (T)</button>
                    <button id="tb-btn-rot" class="tb-btn" title="Rotate (R)">Rotate (R)</button>
                  </div>
                </div>
                <div id="part-panel-mount"></div>
              </div>
            </details>
            <details class="op-section">
              <summary>CAD properties</summary>
              <div class="op-section-body">
                <div id="properties-panel-mount"></div>
                <div id="parts-list-mount"></div>
              </div>
            </details>
            <details class="op-section">
              <summary>Engineer log</summary>
              <div id="log-mount" class="log-mount"></div>
            </details>
            <details class="op-section">
              <summary>Display</summary>
              <div class="op-section-body op-row">
                <span>Dark mode</span>
                <label class="op-switch"><input type="checkbox" id="op-dark" ${this.savedTheme() === 'dark' ? 'checked' : ''}><span></span></label>
              </div>
            </details>
          </div>
        </aside>

        <input type="file" id="cad-file-input" class="hidden-file-input" accept=".step,.stp,.STEP,.STP,.stl,.STL,.obj,.OBJ,*" />

        <div class="error-toast" id="error-toast">
          <div class="error-icon">${icon('warn', 20)}</div>
          <div class="error-content">
            <div class="error-title" id="error-title">Problem</div>
            <div class="error-message" id="error-message"></div>
          </div>
          <button class="btn-close-toast" id="btn-close-toast" aria-label="Close">${icon('close', 16)}</button>
        </div>
      </div>
    `;

    const q = <T extends HTMLElement>(sel: string) => this.root.querySelector<T>(sel)!;
    this.topBarContainer = q('#top-bar-container');
    this.stepperContainer = q('#op-stepper');
    this.autoPickContainer = q('#auto-pick-mount');
    this.activityContainer = q('#op-activity');
    this.viewChipContainer = q('#op-view-chip');
    this.settingsRobotContainer = q('#op-settings-robot');
    this.partsListContainer = q('#parts-list-mount');
    this.viewportContainer = q('#viewport-container');
    this.gripperPanelContainer = q('#gripper-panel-mount');
    this.partPanelContainer = q('#part-panel-mount');
    this.propertiesPanelContainer = q('#properties-panel-mount');
    this.logContainer = q('#log-mount');

    this.fileInput = q('#cad-file-input');
    this.jobText = q('#op-job');
    this.gripperStatusBadge = q('#gripper-status-badge');
    this.dragOverlay = q('#drag-overlay');
    this.loadingOverlay = q('#loading-overlay');
    this.loadingMessage = q('#loading-message');
    this.errorToast = q('#error-toast');
    this.errorMessage = q('#error-message');
    this.errorTitle = q('#error-title');
    this.drawer = q('#op-drawer');
    this.scrim = q('#op-scrim');

    this.btnTbEditGripper = q('#tb-btn-edit-gripper');
    this.btnTbEditPart = q('#tb-btn-edit-part');
    this.btnTbTrans = q('#tb-btn-trans');
    this.btnTbRot = q('#tb-btn-rot');
  }

  private bindEvents() {
    const on = (sel: string, ev: string, fn: (e: Event) => void) => this.root.querySelector(sel)?.addEventListener(ev, fn);

    // Stop robot: always enabled, never waits for anything else.
    on('#op-stop', 'click', () => this.callbacks.onStop());
    on('#op-reset-view', 'click', () => this.callbacks.onResetView());
    on('#op-top-view', 'click', () => this.callbacks.onTopView());
    on('#op-help', 'click', (e) => (e.currentTarget as HTMLElement).classList.toggle('show-tip'));

    on('#op-settings-btn', 'click', () => this.openSettings(true));
    on('#op-drawer-close', 'click', () => this.openSettings(false));
    on('#op-scrim', 'click', () => this.openSettings(false));
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && this.drawer.classList.contains('open')) this.openSettings(false);
    });
    on('#op-dark', 'change', (e) => this.applyTheme((e.target as HTMLInputElement).checked ? 'dark' : 'light', true));

    on('#btn-load-gripper', 'click', () => this.callbacks.onLoadGripperClick());

    this.fileInput.addEventListener('change', () => {
      const file = this.fileInput.files?.[0];
      if (file) this.handleFile(file);
    });

    this.btnTbEditGripper.addEventListener('click', () => {
      this.setActiveTarget('gripper');
      this.callbacks.onTargetToggle?.('gripper');
    });
    this.btnTbEditPart.addEventListener('click', () => {
      this.setActiveTarget('part');
      this.callbacks.onTargetToggle?.('part');
    });
    this.btnTbTrans.addEventListener('click', () => {
      this.setTransformMode('translate');
      this.callbacks.onTransformModeToggle('translate');
    });
    this.btnTbRot.addEventListener('click', () => {
      this.setTransformMode('rotate');
      this.callbacks.onTransformModeToggle('rotate');
    });

    // Drag and drop onto the 3D view
    let dragCounter = 0;
    const vp = this.viewportContainer;
    vp.addEventListener('dragenter', (e) => {
      e.preventDefault();
      dragCounter++;
      this.dragOverlay.classList.add('visible');
    });
    vp.addEventListener('dragover', (e) => e.preventDefault());
    vp.addEventListener('dragleave', (e) => {
      e.preventDefault();
      if (--dragCounter <= 0) {
        dragCounter = 0;
        this.dragOverlay.classList.remove('visible');
      }
    });
    vp.addEventListener('drop', (e) => {
      e.preventDefault();
      dragCounter = 0;
      this.dragOverlay.classList.remove('visible');
      const file = e.dataTransfer?.files[0];
      if (file) this.handleFile(file);
    });

    on('#btn-close-toast', 'click', () => this.errorToast.classList.remove('visible'));
  }

  /** Opens the file chooser (Load part step, File > Open Part). */
  public pickFile() {
    this.fileInput.value = '';
    this.fileInput.click();
  }

  /** A file chosen or dropped anywhere: images are refused, the rest goes to the loader. */
  public handleFile(file: File) {
    if (/\.(jpe?g|png|gif|webp|bmp)$/i.test(file.name)) {
      this.showError(`"${file.name}" is a picture, not a part. Use a STEP file.`, 'Load part');
      return;
    }
    this.callbacks.onFileSelected(file);
  }

  public openSettings(open: boolean) {
    this.drawer.classList.toggle('open', open);
    this.scrim.classList.toggle('open', open);
    this.drawer.setAttribute('aria-hidden', open ? 'false' : 'true');
  }

  private savedTheme(): 'light' | 'dark' {
    try {
      return localStorage.getItem(THEME_KEY) === 'dark' ? 'dark' : 'light';
    } catch {
      return 'light';
    }
  }

  private applyTheme(theme: 'light' | 'dark', save = false) {
    document.documentElement.dataset.theme = theme;
    if (!save) return;
    try {
      localStorage.setItem(THEME_KEY, theme);
    } catch {
      // ignore
    }
  }

  /** Robot / gripper status pills. Red ones show the one-line fix. */
  public setPills(p: Pills) {
    const set = (id: string, pill: Pill) => {
      const el = this.root.querySelector<HTMLElement>(id)!;
      el.dataset.state = pill.state;
      el.title = pill.text;
      el.querySelector('.op-pill-icon')!.innerHTML = icon(
        pill.state === 'ok' ? 'checkCircle' : pill.state === 'error' ? 'error' : pill.state === 'warn' ? 'warn' : 'info',
        16
      );
      el.querySelector('.op-pill-text')!.textContent = pill.text;
    };
    set('#pill-robot', p.robot);
    set('#pill-gripper', p.gripper);
  }

  public setStation(n: number) {
    this.station = n;
    this.updateJob();
  }

  private updateJob() {
    const what = this.selection
      ? `Fixture ${this.selection.fixture} · ${this.selection.part}`
      : `Job: ${this.partName ?? 'no part loaded'}`;
    this.jobText.innerHTML = `${escapeHtml(what)} · Station ${this.station}`;
  }

  /** The fixture and part in use (fixture library), or null for a file opened directly. */
  public setSelection(fixture: string | null, part: string | null) {
    this.selection = fixture && part ? { fixture, part } : null;
    this.updateJob();
  }

  public setActiveTarget(target: 'part' | 'gripper') {
    this.btnTbEditGripper.classList.toggle('active', target === 'gripper');
    this.btnTbEditPart.classList.toggle('active', target === 'part');
  }

  public setTransformMode(mode: 'translate' | 'rotate') {
    this.btnTbTrans.classList.toggle('active', mode === 'translate');
    this.btnTbRot.classList.toggle('active', mode === 'rotate');
  }

  public setGripperBadge(loaded: boolean) {
    this.gripperStatusBadge.classList.toggle('active', loaded);
  }

  public setActiveGripper(id: GripperId) {
    const text = this.gripperStatusBadge.querySelector('.gripper-text');
    if (text) text.textContent = GRIPPERS[id].label;
  }

  public setLoadedFileName(name: string) {
    this.partName = name;
    this.updateJob();
  }

  public showLoading(isLoading: boolean, message?: string) {
    if (message) this.loadingMessage.textContent = message;
    this.loadingOverlay.classList.toggle('visible', isLoading);
  }

  public showError(message: string, title = 'Problem') {
    this.showToast(title, message, 'error');
  }

  public showInfo(title: string, message: string, autoHideMs?: number) {
    this.showToast(title, message, 'info');
    if (autoHideMs) this.toastTimer = window.setTimeout(() => this.hideToast(), autoHideMs);
  }

  public hideToast() {
    this.errorToast.classList.remove('visible');
  }

  private toastTimer: number | null = null;

  /** Where messages go instead of the pop-up once the log exists (set by main.ts). */
  public logSink: ((text: string, kind: 'info' | 'error') => void) | null = null;

  private showToast(title: string, message: string, variant: 'error' | 'info') {
    if (this.logSink) {
      this.logSink(`${title}: ${message}`, variant);
      return;
    }
    if (this.toastTimer !== null) {
      window.clearTimeout(this.toastTimer);
      this.toastTimer = null;
    }
    this.errorTitle.textContent = title;
    this.errorMessage.textContent = message;
    this.errorToast.classList.toggle('is-info', variant === 'info');
    this.errorToast.classList.add('visible');
  }
}
