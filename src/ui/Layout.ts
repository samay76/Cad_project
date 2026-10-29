export interface LayoutCallbacks {
  onFileSelected: (file: File) => void;
  onSampleSelected: (sampleName: string, sampleUrl: string) => void;
  onUrlSelected: (url: string) => void;
  onLoadGripperClick: () => void;
  onTransformModeToggle: (mode: 'translate' | 'rotate') => void;
  onSnapPointClick: () => void;
  onFlipApproachClick: () => void;
  onTestGraspClick: () => void;
  onStopTestClick: () => void;
}

export class AppLayout {
  private root: HTMLElement;
  private callbacks: LayoutCallbacks;

  // DOM Container references
  public topBarContainer!: HTMLElement;
  public partsListContainer!: HTMLElement;
  public graspListContainer!: HTMLElement;
  public viewportContainer!: HTMLElement;
  public gripperPanelContainer!: HTMLElement;
  public propertiesPanelContainer!: HTMLElement;

  private fileInput!: HTMLInputElement;
  private fileNameDisplay!: HTMLElement;
  private gripperStatusBadge!: HTMLElement;
  private dragOverlay!: HTMLElement;
  private loadingOverlay!: HTMLElement;
  private loadingMessage!: HTMLElement;
  private errorToast!: HTMLElement;
  private errorMessage!: HTMLElement;

  // Floating toolbar buttons
  private btnTbTrans!: HTMLButtonElement;
  private btnTbRot!: HTMLButtonElement;
  private btnTbSnap!: HTMLButtonElement;
  private btnTbFlip!: HTMLButtonElement;
  private btnTbTest!: HTMLButtonElement;
  private btnTbStop!: HTMLButtonElement;

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
              <button id="btn-load-gripper" class="btn btn-gripper" title="Load Robotiq 2F-85 Gripper from /gripper/">
                <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <rect x="7" y="11" width="10" height="9" rx="1"/>
                  <path d="M12 2v5M9 7h6M8 11V7M16 11V7"/>
                  <circle cx="10" cy="15" r="1"/>
                  <circle cx="14" cy="15" r="1"/>
                </svg>
                <span>Load Gripper</span>
              </button>

              <!-- Direct STEP File URL / Path Input -->
              <div class="url-input-group">
                <input type="text" id="cad-url-input" class="cad-url-input" placeholder="Or enter STEP file URL / path..." />
                <button id="btn-load-url" class="btn btn-secondary btn-sm" title="Load STEP file from URL or path">
                  <span>Load</span>
                </button>
              </div>

              <!-- Quick Sample Models Dropdown -->
              <div class="sample-dropdown">
                <button id="btn-samples-menu" class="btn btn-outline" title="Load sample CAD files">
                  <span>Samples</span>
                  <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                    <polyline points="6 9 12 15 18 9"/>
                  </svg>
                </button>
                <div class="dropdown-menu" id="samples-dropdown-menu">
                  <button class="dropdown-item" data-url="/samples/skf_cam_roller_305703_c-2z_0.stp" data-name="skf_cam_roller_305703_c-2z_0.stp">
                    <span class="file-icon">⚙️</span>
                    <div class="item-text">
                      <span class="item-title">SKF Cam Roller (STEP)</span>
                      <span class="item-desc">skf_cam_roller_305703_c-2z_0.stp</span>
                    </div>
                  </button>
                  <button class="dropdown-item" data-url="/samples/as1_pe_203.stp" data-name="as1_pe_203.stp">
                    <span class="file-icon">📦</span>
                    <div class="item-text">
                      <span class="item-title">as1_pe_203.stp</span>
                      <span class="item-desc">Assembly (18 Parts, STEP)</span>
                    </div>
                  </button>
                  <button class="dropdown-item" data-url="/samples/as1-oc-214.stp" data-name="as1-oc-214.stp">
                    <span class="file-icon">⚙️</span>
                    <div class="item-text">
                      <span class="item-title">as1-oc-214.stp</span>
                      <span class="item-desc">AP214 Assembly (STEP)</span>
                    </div>
                  </button>
                  <button class="dropdown-item" data-url="/samples/dm1-oc-214.stp" data-name="dm1-oc-214.stp">
                    <span class="file-icon">🔩</span>
                    <div class="item-text">
                      <span class="item-title">dm1-oc-214.stp</span>
                      <span class="item-desc">Machined Part (STEP)</span>
                    </div>
                  </button>
                  <button class="dropdown-item" data-url="/samples/cube.stp" data-name="cube.stp">
                    <span class="file-icon">🧊</span>
                    <div class="item-text">
                      <span class="item-title">cube.stp</span>
                      <span class="item-desc">Simple Cube (STEP)</span>
                    </div>
                  </button>
                  <button class="dropdown-item" data-url="/samples/cube.stl" data-name="cube.stl">
                    <span class="file-icon">🔺</span>
                    <div class="item-text">
                      <span class="item-title">cube.stl</span>
                      <span class="item-desc">Cube (STL Mesh)</span>
                    </div>
                  </button>
                  <button class="dropdown-item" data-url="/samples/cube.obj" data-name="cube.obj">
                    <span class="file-icon">📐</span>
                    <div class="item-text">
                      <span class="item-title">cube.obj</span>
                      <span class="item-desc">Cube (OBJ Format)</span>
                    </div>
                  </button>
                </div>
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
            <div id="parts-list-mount"></div>
            <div id="grasps-list-mount"></div>
          </aside>

          <!-- 3D VIEWPORT CENTER (Takes remaining space) -->
          <section class="viewport-section" id="viewport-container">
            <!-- VIEWPORT FLOATING TOOLBAR -->
            <div class="viewport-toolbar" id="viewport-toolbar">
              <div class="toolbar-group">
                <button id="tb-btn-trans" class="tb-btn active" title="Translate Gripper (T)">
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
                <button id="tb-btn-rot" class="tb-btn" title="Rotate Gripper (R)">
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

              <button id="tb-btn-test" class="tb-btn tb-btn-primary" title="Test Grasp Sequence">
                <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <polygon points="5 3 19 12 5 21 5 3"/>
                </svg>
                <span>Test Grasp</span>
              </button>

              <button id="tb-btn-stop" class="tb-btn tb-btn-danger" title="Stop Test Sequence" disabled>
                <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <rect x="6" y="6" width="12" height="12"/>
                </svg>
                <span>Stop</span>
              </button>
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
              <button class="right-tab-btn" data-tab="cad">CAD Properties</button>
            </div>
            <div class="right-tab-content active" id="tab-gripper-content">
              <div id="gripper-panel-mount"></div>
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
            <div class="error-title">CAD Loading Error</div>
            <div class="error-message" id="error-message">Failed to load CAD file.</div>
          </div>
          <button class="btn-close-toast" id="btn-close-toast">✕</button>
        </div>
      </div>
    `;

    this.topBarContainer = this.root.querySelector('#top-bar-container')!;
    this.partsListContainer = this.root.querySelector('#parts-list-mount')!;
    this.graspListContainer = this.root.querySelector('#grasps-list-mount')!;
    this.viewportContainer = this.root.querySelector('#viewport-container')!;
    this.gripperPanelContainer = this.root.querySelector('#gripper-panel-mount')!;
    this.propertiesPanelContainer = this.root.querySelector('#properties-panel-mount')!;

    this.fileInput = this.root.querySelector('#cad-file-input')!;
    this.fileNameDisplay = this.root.querySelector('#loaded-file-text')!;
    this.gripperStatusBadge = this.root.querySelector('#gripper-status-badge')!;
    this.dragOverlay = this.root.querySelector('#drag-overlay')!;
    this.loadingOverlay = this.root.querySelector('#loading-overlay')!;
    this.loadingMessage = this.root.querySelector('#loading-message')!;
    this.errorToast = this.root.querySelector('#error-toast')!;
    this.errorMessage = this.root.querySelector('#error-message')!;

    this.btnTbTrans = this.root.querySelector('#tb-btn-trans')!;
    this.btnTbRot = this.root.querySelector('#tb-btn-rot')!;
    this.btnTbSnap = this.root.querySelector('#tb-btn-snap')!;
    this.btnTbFlip = this.root.querySelector('#tb-btn-flip')!;
    this.btnTbTest = this.root.querySelector('#tb-btn-test')!;
    this.btnTbStop = this.root.querySelector('#tb-btn-stop')!;
  }

  private bindEvents() {
    const loadBtn = this.root.querySelector('#btn-load-cad');
    if (loadBtn) {
      loadBtn.addEventListener('click', () => {
        this.fileInput.value = '';
        this.fileInput.click();
      });
    }

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

    // STEP File URL / Path Input
    const urlInput = this.root.querySelector<HTMLInputElement>('#cad-url-input');
    const loadUrlBtn = this.root.querySelector<HTMLButtonElement>('#btn-load-url');
    const triggerUrlLoad = () => {
      const url = urlInput?.value.trim();
      if (url) {
        this.callbacks.onUrlSelected(url);
      }
    };

    if (loadUrlBtn) {
      loadUrlBtn.addEventListener('click', triggerUrlLoad);
    }
    if (urlInput) {
      urlInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
          triggerUrlLoad();
        }
      });
    }

    // Sample Models Dropdown
    const samplesBtn = this.root.querySelector('#btn-samples-menu');
    const dropdownMenu = this.root.querySelector('#samples-dropdown-menu');
    if (samplesBtn && dropdownMenu) {
      samplesBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        dropdownMenu.classList.toggle('open');
      });

      document.addEventListener('click', () => {
        dropdownMenu.classList.remove('open');
      });

      const sampleItems = dropdownMenu.querySelectorAll<HTMLButtonElement>('.dropdown-item');
      sampleItems.forEach((item) => {
        item.addEventListener('click', (e) => {
          e.stopPropagation();
          dropdownMenu.classList.remove('open');
          const url = item.dataset.url;
          const name = item.dataset.name;
          if (url && name) {
            this.callbacks.onSampleSelected(name, url);
          }
        });
      });
    }

    // Viewport Floating Toolbar Events
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

    this.btnTbTest.addEventListener('click', () => {
      this.callbacks.onTestGraspClick();
    });

    this.btnTbStop.addEventListener('click', () => {
      this.callbacks.onStopTestClick();
    });

    // Right Sidebar Tab Switcher
    const rightTabBtns = this.root.querySelectorAll<HTMLButtonElement>('.right-tab-btn');
    const tabGripper = this.root.querySelector('#tab-gripper-content');
    const tabCad = this.root.querySelector('#tab-cad-content');

    rightTabBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        rightTabBtns.forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        const tab = btn.dataset.tab;
        if (tab === 'gripper') {
          tabGripper?.classList.add('active');
          tabCad?.classList.remove('active');
        } else {
          tabCad?.classList.add('active');
          tabGripper?.classList.remove('active');
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

  public setTransformMode(mode: 'translate' | 'rotate') {
    this.btnTbTrans.classList.toggle('active', mode === 'translate');
    this.btnTbRot.classList.toggle('active', mode === 'rotate');
  }

  public setCanSnap(canSnap: boolean) {
    this.btnTbSnap.disabled = !canSnap;
  }

  public setTestGraspActive(testing: boolean) {
    this.btnTbTest.disabled = testing;
    this.btnTbStop.disabled = !testing;
  }

  public setGripperBadge(loaded: boolean) {
    this.gripperStatusBadge.classList.toggle('active', loaded);
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

  public showError(message: string) {
    this.errorMessage.textContent = message;
    this.errorToast.classList.add('visible');
  }
}
