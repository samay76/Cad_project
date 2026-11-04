import { GRIPPERS, ROBOTS, type GripperId, type RobotId } from '../config';

export interface LayoutCallbacks {
  onFileSelected: (file: File) => void;
  onLoadGripperClick: () => void;
  onGripperSelect: (id: GripperId) => void;
  onRobotSelect: (id: RobotId) => void;
  onTransformModeToggle: (mode: 'translate' | 'rotate') => void;
  onTargetToggle?: (target: 'part' | 'gripper') => void;
  onSnapPointClick: () => void;
  onFlipApproachClick: () => void;
}

export class AppLayout {
  private root: HTMLElement;
  private callbacks: LayoutCallbacks;

  // DOM Container references
  public topBarContainer!: HTMLElement;
  public autoPickContainer!: HTMLElement;
  public partsListContainer!: HTMLElement;
  public graspListContainer!: HTMLElement;
  public viewportContainer!: HTMLElement;
  public gripperPanelContainer!: HTMLElement;
  public partPanelContainer!: HTMLElement;
  public propertiesPanelContainer!: HTMLElement;

  private fileInput!: HTMLInputElement;
  private fileNameDisplay!: HTMLElement;
  private gripperStatusBadge!: HTMLElement;
  private dragOverlay!: HTMLElement;
  private loadingOverlay!: HTMLElement;
  private loadingMessage!: HTMLElement;
  private errorToast!: HTMLElement;
  private errorMessage!: HTMLElement;
  private errorTitle!: HTMLElement;

  // Floating toolbar buttons
  private btnTbEditGripper!: HTMLButtonElement;
  private btnTbEditPart!: HTMLButtonElement;
  private btnTbTrans!: HTMLButtonElement;
  private btnTbRot!: HTMLButtonElement;
  private btnTbSnap!: HTMLButtonElement;
  private btnTbFlip!: HTMLButtonElement;

  constructor(root: HTMLElement, callbacks: LayoutCallbacks) {
    this.root = root;
    this.callbacks = callbacks;
    this.initDOM();
    this.bindEvents();
  }

  private initDOM() {
    this.root.innerHTML = `
      <div class="app-layout">
        <!-- TOP BAR -->
        <header class="app-header" id="top-bar-container">
          <div class="header-left">
            <div class="brand">
              <div class="brand-icon">
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2">
                  <path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"/>
                  <polyline points="3.27 6.96 12 12.01 20.73 6.96"/>
                  <line x1="12" y1="22.08" x2="12" y2="12"/>
                </svg>
              </div>
              <div class="brand-text">
                <span class="brand-title">COBOT PICK-POINT</span>
                <span class="brand-subtitle">CAD & Gripper Pose Tool (Step 2)</span>
              </div>
            </div>

            <div class="header-actions">
              <input type="file" id="cad-file-input" class="hidden-file-input" accept=".step,.stp,.STEP,.STP,.stl,.STL,.obj,.OBJ,.iges,.igs,.IGES,.IGS,*" />
              <button id="btn-load-cad" class="btn btn-primary" title="Open local CAD file (.step, .stp, .stl, .obj)">
                <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
                  <polyline points="17 8 12 3 7 8"/>
                  <line x1="12" y1="3" x2="12" y2="15"/>
                </svg>
                <span>Load CAD</span>
              </button>

              <!-- LOAD GRIPPER BUTTON -->
              <button id="btn-load-gripper" class="btn btn-gripper" title="Load the selected gripper">
                <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <rect x="7" y="11" width="10" height="9" rx="1"/>
                  <path d="M12 2v5M9 7h6M8 11V7M16 11V7"/>
                  <circle cx="10" cy="15" r="1"/>
                  <circle cx="14" cy="15" r="1"/>
                </svg>
                <span>Load Gripper</span>
              </button>

              <!-- GRIPPER SELECTOR -->
              <div class="gripper-select" role="group" aria-label="Gripper type" title="Select which gripper to use">
                ${Object.values(GRIPPERS)
                  .map(
                    (g) =>
                      `<button class="gripper-select-btn" data-gripper="${g.id}" title="Use the ${g.label} (opening 0-${g.maxOpeningWidth} mm)">${g.label.replace('Robotiq ', '')}</button>`
                  )
                  .join('')}
              </div>

              <!-- ROBOT ARM SELECTOR (used by the MuJoCo simulation) -->
              <div class="gripper-select robot-select" role="group" aria-label="Robot arm" title="Robot arm used in the MuJoCo simulation">
                <span class="robot-select-label">Robot</span>
                ${Object.values(ROBOTS)
                  .map(
                    (r) =>
                      `<button class="gripper-select-btn robot-select-btn" data-robot="${r.id}" title="Simulate with the ${r.label} (${r.payloadKg} kg payload, ${r.reachMm} mm reach)">${r.label.replace('Elite ', '')}</button>`
                  )
                  .join('')}
              </div>

              <!-- Loaded File Name Display -->
              <div id="file-name-badge" class="file-badge">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"/>
                  <polyline points="14 2 14 8 20 8"/>
                </svg>
                <span class="badge-text" id="loaded-file-text">No CAD loaded</span>
              </div>
            </div>
          </div>

          <div class="header-right">
            <!-- Gripper Status Badge -->
            <div class="gripper-badge" id="gripper-status-badge">
              <span class="gripper-dot"></span>
              <span class="gripper-text">Robotiq 2F-85 Active</span>
            </div>

            <div class="status-indicator" id="app-status">
              <span class="status-dot"></span>
              <span class="status-text">Z-Up Frame (mm)</span>
            </div>
          </div>
        </header>

        <!-- MAIN THREE-BLOCK WORKSPACE -->
        <main class="app-workspace">
          <!-- LEFT SIDEBAR (~260px) -->
          <aside class="sidebar-left" id="left-sidebar-container">
            <div id="auto-pick-mount"></div>
            <div id="parts-list-mount"></div>
            <div id="grasps-list-mount"></div>
          </aside>

          <!-- 3D VIEWPORT CENTER (Takes remaining space) -->
          <section class="viewport-section" id="viewport-container">
            <!-- VIEWPORT FLOATING TOOLBAR -->
            <div class="viewport-toolbar" id="viewport-toolbar">
              <div class="toolbar-target-group">
                <button id="tb-btn-edit-gripper" class="tb-btn tb-btn-target active" title="Edit Gripper Pose (G)">Gripper</button>
                <button id="tb-btn-edit-part" class="tb-btn tb-btn-target" title="Edit CAD Part Pose (P)">Part</button>
              </div>

              <div class="toolbar-divider"></div>

              <div class="toolbar-group">
                <button id="tb-btn-trans" class="tb-btn active" title="Translate (T)">
                  <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                    <polyline points="5 9 2 12 5 15"/>
                    <polyline points="9 5 12 2 15 5"/>
                    <polyline points="15 19 12 22 9 19"/>
                    <polyline points="19 9 22 12 19 15"/>
                    <line x1="2" y1="12" x2="22" y2="12"/>
                    <line x1="12" y1="2" x2="12" y2="22"/>
                  </svg>
                  <span>Move (T)</span>
                </button>
                <button id="tb-btn-rot" class="tb-btn" title="Rotate (R)">
                  <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                    <path d="M21.5 2v6h-6M21.34 15.57a10 10 0 1 1-.57-8.38l5.67-5.67"/>
                  </svg>
                  <span>Rotate (R)</span>
                </button>
              </div>

              <div class="toolbar-divider"></div>

              <button id="tb-btn-snap" class="tb-btn" title="Snap Gripper to Clicked Surface Point" disabled>
                <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <circle cx="12" cy="12" r="3"/>
                  <path d="M12 2v3m0 14v3M2 12h3m14 0h3"/>
                </svg>
                <span>Snap to Point</span>
              </button>

              <button id="tb-btn-flip" class="tb-btn" title="Flip Approach Direction (180°)">
                <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="m3 16 4 4 4-4"/>
                  <path d="M7 20V4"/>
                  <path d="m21 8-4-4-4 4"/>
                  <path d="M17 4v16"/>
                </svg>
                <span>Flip (180°)</span>
              </button>

              <div class="toolbar-divider"></div>


            </div>

            <!-- Drag and Drop Overlay -->
            <div class="drag-drop-overlay" id="drag-overlay">
              <div class="drag-drop-content">
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8">
                  <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
                  <polyline points="17 8 12 3 7 8"/>
                  <line x1="12" y1="3" x2="12" y2="15"/>
                </svg>
                <h3>Drop CAD file here</h3>
                <p>Supports .step, .stp, .stl, .obj</p>
              </div>
            </div>

            <!-- Loading Spinner Overlay -->
            <div class="loading-overlay" id="loading-overlay">
              <div class="loading-spinner"></div>
              <p class="loading-message" id="loading-message">Processing CAD geometry...</p>
            </div>

            <!-- Viewport HUD Overlay Instructions -->
            <div class="viewport-hud">
              <span class="hud-item"><b>Left Drag:</b> Rotate</span>
              <span class="hud-item"><b>Right Drag:</b> Pan</span>
              <span class="hud-item"><b>Scroll:</b> Zoom</span>
              <span class="hud-item"><b>T / R:</b> Translate/Rotate Gizmo</span>
            </div>
          </section>

          <!-- RIGHT SIDEBAR (~300px) -->
          <aside class="sidebar-right" id="right-sidebar-container">
            <div class="right-tab-bar">
              <button class="right-tab-btn active" data-tab="gripper">Gripper & Grasp</button>
              <button class="right-tab-btn" data-tab="part">Part Pose</button>
              <button class="right-tab-btn" data-tab="cad">CAD Properties</button>
            </div>
            <div class="right-tab-content active" id="tab-gripper-content">
              <div id="gripper-panel-mount"></div>
            </div>
            <div class="right-tab-content" id="tab-part-content">
              <div id="part-panel-mount"></div>
            </div>
            <div class="right-tab-content" id="tab-cad-content">
              <div id="properties-panel-mount"></div>
            </div>
          </aside>
        </main>

        <!-- ERROR TOAST NOTIFICATION -->
        <div class="error-toast" id="error-toast">
          <div class="error-icon">⚠️</div>
          <div class="error-content">
            <div class="error-title" id="error-title">CAD Loading Error</div>
            <div class="error-message" id="error-message">Failed to load CAD file.</div>
          </div>
          <button class="btn-close-toast" id="btn-close-toast">✕</button>
        </div>
      </div>
    `;

    this.topBarContainer = this.root.querySelector('#top-bar-container')!;
    this.autoPickContainer = this.root.querySelector('#auto-pick-mount')!;
    this.partsListContainer = this.root.querySelector('#parts-list-mount')!;
    this.graspListContainer = this.root.querySelector('#grasps-list-mount')!;
    this.viewportContainer = this.root.querySelector('#viewport-container')!;
    this.gripperPanelContainer = this.root.querySelector('#gripper-panel-mount')!;
    this.partPanelContainer = this.root.querySelector('#part-panel-mount')!;
    this.propertiesPanelContainer = this.root.querySelector('#properties-panel-mount')!;

    this.fileInput = this.root.querySelector('#cad-file-input')!;
    this.fileNameDisplay = this.root.querySelector('#loaded-file-text')!;
    this.gripperStatusBadge = this.root.querySelector('#gripper-status-badge')!;
    this.dragOverlay = this.root.querySelector('#drag-overlay')!;
    this.loadingOverlay = this.root.querySelector('#loading-overlay')!;
    this.loadingMessage = this.root.querySelector('#loading-message')!;
    this.errorToast = this.root.querySelector('#error-toast')!;
    this.errorMessage = this.root.querySelector('#error-message')!;
    this.errorTitle = this.root.querySelector('#error-title')!;

    this.btnTbEditGripper = this.root.querySelector('#tb-btn-edit-gripper')!;
    this.btnTbEditPart = this.root.querySelector('#tb-btn-edit-part')!;
    this.btnTbTrans = this.root.querySelector('#tb-btn-trans')!;
    this.btnTbRot = this.root.querySelector('#tb-btn-rot')!;
    this.btnTbSnap = this.root.querySelector('#tb-btn-snap')!;
    this.btnTbFlip = this.root.querySelector('#tb-btn-flip')!;
  }

  private bindEvents() {
    const loadBtn = this.root.querySelector('#btn-load-cad');
    if (loadBtn) {
      loadBtn.addEventListener('click', () => {
        this.fileInput.value = '';
        this.fileInput.click();
      });
    }

    this.root.querySelectorAll<HTMLButtonElement>('.robot-select-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        if (!btn.classList.contains('is-active')) {
          this.callbacks.onRobotSelect(btn.dataset.robot as RobotId);
        }
      });
    });

    this.root.querySelectorAll<HTMLButtonElement>('.gripper-select-btn[data-gripper]').forEach((btn) => {
      btn.addEventListener('click', () => {
        if (!btn.classList.contains('is-active')) {
          this.callbacks.onGripperSelect(btn.dataset.gripper as GripperId);
        }
      });
    });

    const loadGripperBtn = this.root.querySelector('#btn-load-gripper');
    if (loadGripperBtn) {
      loadGripperBtn.addEventListener('click', () => {
        this.callbacks.onLoadGripperClick();
      });
    }

    this.fileInput.addEventListener('change', () => {
      if (this.fileInput.files && this.fileInput.files.length > 0) {
        const file = this.fileInput.files[0];
        this.callbacks.onFileSelected(file);
      }
    });

    // Viewport Floating Toolbar Events
    // Edit Target Switcher (Gripper vs Part)
    this.btnTbEditGripper?.addEventListener('click', () => {
      this.setActiveTarget('gripper');
      if (this.callbacks.onTargetToggle) {
        this.callbacks.onTargetToggle('gripper');
      }
    });

    this.btnTbEditPart?.addEventListener('click', () => {
      this.setActiveTarget('part');
      if (this.callbacks.onTargetToggle) {
        this.callbacks.onTargetToggle('part');
      }
    });

    this.btnTbTrans.addEventListener('click', () => {
      this.setTransformMode('translate');
      this.callbacks.onTransformModeToggle('translate');
    });

    this.btnTbRot.addEventListener('click', () => {
      this.setTransformMode('rotate');
      this.callbacks.onTransformModeToggle('rotate');
    });

    this.btnTbSnap.addEventListener('click', () => {
      this.callbacks.onSnapPointClick();
    });

    this.btnTbFlip.addEventListener('click', () => {
      this.callbacks.onFlipApproachClick();
    });

    // Right Sidebar Tab Switcher
    const rightTabBtns = this.root.querySelectorAll<HTMLButtonElement>('.right-tab-btn');
    const tabGripper = this.root.querySelector('#tab-gripper-content');
    const tabPart = this.root.querySelector('#tab-part-content');
    const tabCad = this.root.querySelector('#tab-cad-content');

    rightTabBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        rightTabBtns.forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        const tab = btn.dataset.tab;

        tabGripper?.classList.toggle('active', tab === 'gripper');
        tabPart?.classList.toggle('active', tab === 'part');
        tabCad?.classList.toggle('active', tab === 'cad');

        if (tab === 'part' && this.callbacks.onTargetToggle) {
          this.setActiveTarget('part');
          this.callbacks.onTargetToggle('part');
        } else if (tab === 'gripper' && this.callbacks.onTargetToggle) {
          this.setActiveTarget('gripper');
          this.callbacks.onTargetToggle('gripper');
        }
      });
    });

    // Drag and Drop onto Viewport Container
    let dragCounter = 0;
    this.viewportContainer.addEventListener('dragenter', (e) => {
      e.preventDefault();
      e.stopPropagation();
      dragCounter++;
      this.dragOverlay.classList.add('visible');
    });

    this.viewportContainer.addEventListener('dragover', (e) => {
      e.preventDefault();
      e.stopPropagation();
    });

    this.viewportContainer.addEventListener('dragleave', (e) => {
      e.preventDefault();
      e.stopPropagation();
      dragCounter--;
      if (dragCounter <= 0) {
        dragCounter = 0;
        this.dragOverlay.classList.remove('visible');
      }
    });

    this.viewportContainer.addEventListener('drop', (e) => {
      e.preventDefault();
      e.stopPropagation();
      dragCounter = 0;
      this.dragOverlay.classList.remove('visible');

      if (e.dataTransfer && e.dataTransfer.files.length > 0) {
        const file = e.dataTransfer.files[0];
        this.callbacks.onFileSelected(file);
      }
    });

    // Error toast dismiss
    const closeToastBtn = this.root.querySelector('#btn-close-toast');
    if (closeToastBtn) {
      closeToastBtn.addEventListener('click', () => {
        this.errorToast.classList.remove('visible');
      });
    }
  }

  public setActiveTarget(target: 'part' | 'gripper') {
    this.btnTbEditGripper?.classList.toggle('active', target === 'gripper');
    this.btnTbEditPart?.classList.toggle('active', target === 'part');

    // Also update tabs if needed
    const rightTabBtns = this.root.querySelectorAll<HTMLButtonElement>('.right-tab-btn');
    const tabGripper = this.root.querySelector('#tab-gripper-content');
    const tabPart = this.root.querySelector('#tab-part-content');

    rightTabBtns.forEach((btn) => {
      if (btn.dataset.tab === target) {
        btn.classList.add('active');
      } else if (btn.dataset.tab === 'gripper' || btn.dataset.tab === 'part') {
        btn.classList.remove('active');
      }
    });

    if (target === 'gripper') {
      tabGripper?.classList.add('active');
      tabPart?.classList.remove('active');
    } else {
      tabPart?.classList.add('active');
      tabGripper?.classList.remove('active');
    }
  }

  public setTransformMode(mode: 'translate' | 'rotate') {
    this.btnTbTrans.classList.toggle('active', mode === 'translate');
    this.btnTbRot.classList.toggle('active', mode === 'rotate');
  }

  public setCanSnap(canSnap: boolean) {
    this.btnTbSnap.disabled = !canSnap;
  }

  public setGripperBadge(loaded: boolean) {
    this.gripperStatusBadge.classList.toggle('active', loaded);
  }

  /** Highlights the robot arm used for the MuJoCo simulation. */
  public setActiveRobot(id: RobotId) {
    this.root.querySelectorAll<HTMLButtonElement>('.robot-select-btn').forEach((btn) => {
      btn.classList.toggle('is-active', btn.dataset.robot === id);
    });
  }

  /** Highlights the selected gripper and names it in the status badge. */
  public setActiveGripper(id: GripperId) {
    this.root.querySelectorAll<HTMLButtonElement>('.gripper-select-btn[data-gripper]').forEach((btn) => {
      btn.classList.toggle('is-active', btn.dataset.gripper === id);
    });
    const text = this.gripperStatusBadge.querySelector('.gripper-text');
    if (text) text.textContent = `${GRIPPERS[id].label} Active`;
    const loadBtn = this.root.querySelector<HTMLButtonElement>('#btn-load-gripper');
    if (loadBtn) loadBtn.title = `Load the ${GRIPPERS[id].label} gripper`;
  }

  public setLoadedFileName(name: string) {
    this.fileNameDisplay.textContent = name;
    this.fileNameDisplay.parentElement?.classList.add('has-file');
  }

  public showLoading(isLoading: boolean, message?: string) {
    if (isLoading) {
      if (message) this.loadingMessage.textContent = message;
      this.loadingOverlay.classList.add('visible');
    } else {
      this.loadingOverlay.classList.remove('visible');
    }
  }

  public showError(message: string, title = 'CAD Loading Error') {
    this.showToast(title, message, 'error');
  }

  /** autoHideMs: close the message by itself after that time (short confirmations). */
  public showInfo(title: string, message: string, autoHideMs?: number) {
    this.showToast(title, message, 'info');
    if (autoHideMs) {
      this.toastTimer = window.setTimeout(() => this.hideToast(), autoHideMs);
    }
  }

  public hideToast() {
    this.errorToast.classList.remove('visible');
  }

  private toastTimer: number | null = null;

  private showToast(title: string, message: string, variant: 'error' | 'info') {
    if (this.toastTimer !== null) {
      window.clearTimeout(this.toastTimer);
      this.toastTimer = null;
    }
    this.errorTitle.textContent = title;
    this.errorMessage.textContent = message;
    this.errorToast.classList.toggle('is-info', variant === 'info');
    this.errorToast.querySelector('.error-icon')!.textContent = variant === 'info' ? 'ℹ️' : '⚠️';
    this.errorToast.classList.add('visible');
  }
}
