export interface LayoutCallbacks {
  onFileSelected: (file: File) => void;
  onSampleSelected: (sampleName: string, sampleUrl: string) => void;
  onUrlSelected: (url: string) => void;
}

export class AppLayout {
  private root: HTMLElement;
  private callbacks: LayoutCallbacks;

  // DOM Container references
  public topBarContainer!: HTMLElement;
  public leftSidebarContainer!: HTMLElement;
  public viewportContainer!: HTMLElement;
  public rightSidebarContainer!: HTMLElement;

  private fileInput!: HTMLInputElement;
  private fileNameDisplay!: HTMLElement;
  private dragOverlay!: HTMLElement;
  private loadingOverlay!: HTMLElement;
  private loadingMessage!: HTMLElement;
  private errorToast!: HTMLElement;
  private errorMessage!: HTMLElement;

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
                <span class="brand-subtitle">CAD Viewer & Surface Selection</span>
              </div>
            </div>

            <div class="header-actions">
              <input type="file" id="cad-file-input" class="hidden-file-input" accept=".step,.stp,.STEP,.STP,.stl,.STL,.obj,.OBJ,.iges,.igs,.IGES,.IGS,*" />
              <button id="btn-load-cad" class="btn btn-primary" title="Open local CAD file (.step, .stp, .stl, .obj)">
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
                  <polyline points="17 8 12 3 7 8"/>
                  <line x1="12" y1="3" x2="12" y2="15"/>
                </svg>
                <span>Load CAD File</span>
              </button>

              <!-- Direct STEP File URL / Path Input -->
              <div class="url-input-group">
                <input type="text" id="cad-url-input" class="cad-url-input" placeholder="Or enter STEP file URL / path..." />
                <button id="btn-load-url" class="btn btn-secondary btn-sm" title="Load STEP file from URL or path">
                  <span>Load</span>
                </button>
              </div>

              <!-- Quick Sample Models Dropdown for immediate testing -->
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
            <div class="status-indicator" id="app-status">
              <span class="status-dot"></span>
              <span class="status-text">Z-Up Robotics Frame</span>
            </div>
          </div>
        </header>

        <!-- MAIN THREE-BLOCK WORKSPACE -->
        <main class="app-workspace">
          <!-- LEFT SIDEBAR (~260px) -->
          <aside class="sidebar-left" id="left-sidebar-container"></aside>

          <!-- 3D VIEWPORT CENTER (Takes remaining space) -->
          <section class="viewport-section" id="viewport-container">
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
              <span class="hud-item"><b>Click:</b> Select Pick Point</span>
            </div>
          </section>

          <!-- RIGHT SIDEBAR (~300px) -->
          <aside class="sidebar-right" id="right-sidebar-container"></aside>
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
    this.leftSidebarContainer = this.root.querySelector('#left-sidebar-container')!;
    this.viewportContainer = this.root.querySelector('#viewport-container')!;
    this.rightSidebarContainer = this.root.querySelector('#right-sidebar-container')!;

    this.fileInput = this.root.querySelector('#cad-file-input')!;
    this.fileNameDisplay = this.root.querySelector('#loaded-file-text')!;
    this.dragOverlay = this.root.querySelector('#drag-overlay')!;
    this.loadingOverlay = this.root.querySelector('#loading-overlay')!;
    this.loadingMessage = this.root.querySelector('#loading-message')!;
    this.errorToast = this.root.querySelector('#error-toast')!;
    this.errorMessage = this.root.querySelector('#error-message')!;
  }

  private bindEvents() {
    const loadBtn = this.root.querySelector('#btn-load-cad');
    if (loadBtn) {
      loadBtn.addEventListener('click', () => {
        this.fileInput.value = '';
        this.fileInput.click();
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
