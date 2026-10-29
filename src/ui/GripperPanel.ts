import type { GripperState, EulerAngles } from '../types';

export interface GripperPanelEvents {
  onNumericPoseChange: (pos: { x: number; y: number; z: number }, rpy: EulerAngles) => void;
  onStepTranslate: (axis: 'x' | 'y' | 'z', delta: number) => void;
  onStepRotate: (axis: 'x' | 'y' | 'z', delta: number) => void;
  onOpeningChange: (width: number) => void;
  onTcpOffsetChange: (offset: number) => void;
  onPreGraspDistanceChange: (dist: number) => void;
  onSnapToClickedPoint: () => void;
  onFlipApproach: () => void;
  onRotateApproach: (deg: number) => void;
  onResetPose: () => void;
  onTestGrasp: () => void;
  onStopTest: () => void;
  onSaveGrasp: () => void;
  onCloseUntilContact: (tolerance: number) => void;
  onOpenToClear: () => void;
  onAutoPlaceOnOuterWall: () => void;
  onStandoffChange: (standoff: number) => void;
  onDepthChange: (depth: number) => void;
  onToleranceChange: (tolerance: number) => void;
}

export class GripperPanel {
  private container: HTMLElement;
  private events: GripperPanelEvents;
  private state: GripperState | null = null;
  private canSnap = false;

  // Selected step sizes
  private linearStep = 5; // mm
  private angularStep = 15; // deg

  // Config parameters
  private standoff = 0; // mm
  private depth = 15; // mm
  private tolerance = 0.1; // mm

  constructor(container: HTMLElement, events: GripperPanelEvents) {
    this.container = container;
    this.events = events;
    this.render();
  }

  public setGripperState(state: GripperState) {
    this.state = state;
    this.updateValues();
  }

  public setCanSnap(canSnap: boolean) {
    this.canSnap = canSnap;
    const snapBtn = this.container.querySelector<HTMLButtonElement>('#btn-snap-point');
    if (snapBtn) {
      snapBtn.disabled = !canSnap;
      snapBtn.title = canSnap
        ? 'Snap TCP to selected surface point'
        : 'Click a surface point on the CAD model first';
    }
  }

  private render() {
    this.container.innerHTML = `
      <div class="sidebar-block gripper-panel-block">
        <div class="block-header">
          <div class="title-with-badge">
            <span class="block-title">GRIPPER POSE & TCP</span>
            <span id="collision-badge" class="badge-status badge-no-collision">No collision</span>
          </div>
        </div>

        <div class="gripper-scrollable">
          <!-- 1. CONTACT GRIP & OUTER WALL DETECTION -->
          <div class="property-section contact-grip-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M18 10h-1.26A8 8 0 1 0 9 20h9a5 5 0 0 0 0-10z"/>
              </svg>
              <span>Contact Grip (No Penetration)</span>
            </div>

            <!-- Readouts & Feedback -->
            <div class="contact-status-card">
              <div class="contact-status-header">
                <span class="sub-label">Contact Status:</span>
                <span id="contact-status-text" class="contact-status-badge badge-contact-none">No contact</span>
              </div>
              <div id="contact-message-text" class="contact-message-text">Fingers open and ready</div>
              <div class="contact-metrics-row">
                <div class="metric-item">
                  <span class="metric-label">Grasp Width:</span>
                  <span id="label-measured-width" class="metric-value mono">85.0 mm</span>
                </div>
                <div class="metric-item">
                  <span class="metric-label">Fingers Touched:</span>
                  <span id="label-fingers-touched" class="metric-value mono">None</span>
                </div>
              </div>
            </div>

            <!-- Contact Grip Action Buttons -->
            <div class="contact-actions-grid">
              <button id="btn-close-contact" class="btn btn-primary btn-sm" title="Close fingers in 0.5mm steps until outer wall contact, stopping with gap < 0.1mm">
                Close (until contact)
              </button>
              <button id="btn-open-clear" class="btn btn-secondary btn-sm" title="Open fingers fully and verify clear of CAD part">
                Open to clear
              </button>
            </div>

            <button id="btn-auto-place" class="btn btn-secondary btn-sm btn-full" title="Raycast along closing axis to find outer wall hits and center TCP between walls">
              Auto-place on outer wall
            </button>

            <!-- Standoff, Depth, Tolerance Inputs -->
            <div class="contact-params-grid">
              <div class="num-input-item">
                <label title="Standoff distance from CAD surface">Standoff (mm)</label>
                <input type="number" id="inp-standoff" min="0" max="100" step="1" value="0" />
              </div>
              <div class="num-input-item">
                <label title="Grip depth along approach vector">Depth (mm)</label>
                <input type="number" id="inp-depth" min="0" max="100" step="1" value="15" />
              </div>
              <div class="num-input-item">
                <label title="Contact detection tolerance gap">Tolerance (mm)</label>
                <input type="number" id="inp-tolerance" min="0.01" max="1.0" step="0.05" value="0.1" />
              </div>
            </div>
          </div>

          <!-- 2. TEST GRASP MOTION & STATUS -->
          <div class="property-section test-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <polygon points="5 3 19 12 5 21 5 3"/>
              </svg>
              <span>Test Grasp Motion</span>
              <span id="test-phase-badge" class="test-phase-badge">Idle</span>
            </div>

            <div class="test-buttons-row">
              <button id="btn-test-grasp" class="btn btn-primary btn-test" title="Run Pick & Place sequence: Pre-grasp, Approach with collision stop, Contact-aware Close, Lift CAD model, Place and Release">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <polygon points="5 3 19 12 5 21 5 3"/>
                </svg>
                <span>Test Grasp & Lift</span>
              </button>
              <button id="btn-stop-test" class="btn btn-secondary btn-test" disabled>
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <rect x="6" y="6" width="12" height="12"/>
                </svg>
                <span>Stop</span>
              </button>
            </div>
          </div>

          <!-- 3. QUICK ALIGNMENT & SNAPPING -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <circle cx="12" cy="12" r="10"/>
                <path d="m4.93 4.93 4.24 4.24"/>
                <path d="m14.83 9.17 4.24-4.24"/>
                <path d="m14.83 14.83 4.24 4.24"/>
                <path d="m9.17 14.83-4.24 4.24"/>
              </svg>
              <span>Alignment & Snap</span>
            </div>

            <div class="snap-buttons-grid">
              <button id="btn-snap-point" class="btn btn-secondary btn-snap" ${this.canSnap ? '' : 'disabled'}>
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <circle cx="12" cy="12" r="3"/>
                  <path d="M12 2v3m0 14v3M2 12h3m14 0h3"/>
                </svg>
                <span>Snap to Clicked Point</span>
              </button>
              <button id="btn-flip-approach" class="btn btn-secondary btn-snap">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="m3 16 4 4 4-4"/>
                  <path d="M7 20V4"/>
                  <path d="m21 8-4-4-4 4"/>
                  <path d="M17 4v16"/>
                </svg>
                <span>Flip Approach (180°)</span>
              </button>
            </div>

            <div class="approach-spin-row">
              <span class="sub-label">Spin around Approach:</span>
              <div class="spin-buttons">
                <button class="btn-micro" data-spin="-90">-90°</button>
                <button class="btn-micro" data-spin="-15">-15°</button>
                <button class="btn-micro" data-spin="15">+15°</button>
                <button class="btn-micro" data-spin="90">+90°</button>
              </div>
            </div>
          </div>

          <!-- 4. STEP JOGGING CONTROLS -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <polyline points="5 9 2 12 5 15"/>
                <polyline points="9 5 12 2 15 5"/>
                <polyline points="15 19 12 22 9 19"/>
                <polyline points="19 9 22 12 19 15"/>
                <line x1="2" y1="12" x2="22" y2="12"/>
                <line x1="12" y1="2" x2="12" y2="22"/>
              </svg>
              <span>Step Jogging</span>
            </div>

            <div class="step-sizes-selector">
              <span class="sub-label">Step:</span>
              <div class="segmented-control step-seg">
                <button class="seg-btn step-btn" data-linear="1" data-angular="1">1mm / 1°</button>
                <button class="seg-btn step-btn active" data-linear="5" data-angular="15">5mm / 15°</button>
                <button class="seg-btn step-btn" data-linear="10" data-angular="45">10mm / 45°</button>
                <button class="seg-btn step-btn" data-linear="25" data-angular="90">25mm / 90°</button>
              </div>
            </div>

            <div class="jog-grid">
              <!-- Translation Jog -->
              <div class="jog-group">
                <span class="axis-header">Position (mm)</span>
                <div class="jog-row">
                  <span class="axis-tag axis-x">X</span>
                  <button class="btn-jog" data-axis="x" data-dir="-1">-</button>
                  <button class="btn-jog" data-axis="x" data-dir="1">+</button>
                </div>
                <div class="jog-row">
                  <span class="axis-tag axis-y">Y</span>
                  <button class="btn-jog" data-axis="y" data-dir="-1">-</button>
                  <button class="btn-jog" data-axis="y" data-dir="1">+</button>
                </div>
                <div class="jog-row">
                  <span class="axis-tag axis-z">Z</span>
                  <button class="btn-jog" data-axis="z" data-dir="-1">-</button>
                  <button class="btn-jog" data-axis="z" data-dir="1">+</button>
                </div>
              </div>

              <!-- Rotation Jog -->
              <div class="jog-group">
                <span class="axis-header">Rotation (deg)</span>
                <div class="jog-row">
                  <span class="axis-tag">Rx</span>
                  <button class="btn-rot" data-axis="x" data-dir="-1">-</button>
                  <button class="btn-rot" data-axis="x" data-dir="1">+</button>
                </div>
                <div class="jog-row">
                  <span class="axis-tag">Ry</span>
                  <button class="btn-rot" data-axis="y" data-dir="-1">-</button>
                  <button class="btn-rot" data-axis="y" data-dir="1">+</button>
                </div>
                <div class="jog-row">
                  <span class="axis-tag">Rz</span>
                  <button class="btn-rot" data-axis="z" data-dir="-1">-</button>
                  <button class="btn-rot" data-axis="z" data-dir="1">+</button>
                </div>
              </div>
            </div>
          </div>

          <!-- 5. NUMERIC POSE (TWO-WAY BINDING) -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="4" y="4" width="16" height="16" rx="2" ry="2"/>
                <line x1="9" y1="9" x2="15" y2="15"/>
                <line x1="15" y1="9" x2="9" y2="15"/>
              </svg>
              <span>Gripper Pose (World Frame)</span>
            </div>

            <div class="numeric-inputs-grid">
              <div class="num-input-item">
                <label>X (mm)</label>
                <input type="number" id="inp-pos-x" step="0.5" value="0.00" />
              </div>
              <div class="num-input-item">
                <label>Y (mm)</label>
                <input type="number" id="inp-pos-y" step="0.5" value="0.00" />
              </div>
              <div class="num-input-item">
                <label>Z (mm)</label>
                <input type="number" id="inp-pos-z" step="0.5" value="100.00" />
              </div>
              <div class="num-input-item">
                <label>Roll (°)</label>
                <input type="number" id="inp-rpy-r" step="1" value="0.0" />
              </div>
              <div class="num-input-item">
                <label>Pitch (°)</label>
                <input type="number" id="inp-rpy-p" step="1" value="0.0" />
              </div>
              <div class="num-input-item">
                <label>Yaw (°)</label>
                <input type="number" id="inp-rpy-y" step="1" value="0.0" />
              </div>
            </div>

            <!-- Read-only Gripper relative to part readout -->
            <div class="relative-pose-card">
              <div class="relative-pose-title">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="M12 2v20M17 5H9.5a3.5 3.5 0 0 0 0 7h5a3.5 3.5 0 0 1 0 7H6"/>
                </svg>
                <span>Gripper relative to part (T_part_grasp)</span>
              </div>
              <div class="relative-pose-grid">
                <div class="relative-item"><span class="r-tag">X</span><span id="rel-pos-x" class="r-val">0.00</span></div>
                <div class="relative-item"><span class="r-tag">Y</span><span id="rel-pos-y" class="r-val">0.00</span></div>
                <div class="relative-item"><span class="r-tag">Z</span><span id="rel-pos-z" class="r-val">0.00</span></div>
                <div class="relative-item"><span class="r-tag">Rx</span><span id="rel-rpy-r" class="r-val">0.0°</span></div>
                <div class="relative-item"><span class="r-tag">Ry</span><span id="rel-rpy-p" class="r-val">0.0°</span></div>
                <div class="relative-item"><span class="r-tag">Rz</span><span id="rel-rpy-y" class="r-val">0.0°</span></div>
              </div>
            </div>

            <button id="btn-reset-pose" class="btn btn-secondary btn-sm btn-full">
              Reset Gripper Pose
            </button>
          </div>

          <!-- 6. FINGER OPEN / CLOSE & TCP CONFIG -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M7 11V7a5 5 0 0 1 10 0v4"/>
                <rect x="3" y="11" width="18" height="11" rx="2" ry="2"/>
              </svg>
              <span>Manual Opening & TCP</span>
            </div>

            <!-- Open / Close Slider -->
            <div class="slider-control-group">
              <div class="slider-header">
                <span class="sub-label">Opening Width:</span>
                <span id="label-opening-width" class="slider-value mono">85.0 mm</span>
              </div>
              <input type="range" id="slider-opening" min="0" max="85" step="0.5" value="85" class="range-slider" />
              <div class="slider-quick-buttons">
                <button id="btn-quick-close" class="btn btn-outline btn-micro">Close (0mm)</button>
                <button id="btn-quick-half" class="btn btn-outline btn-micro">Half (42.5mm)</button>
                <button id="btn-quick-open" class="btn btn-outline btn-micro">Open (85mm)</button>
              </div>
            </div>

            <!-- Parameters Grid (TCP Offset & Pre-Grasp) -->
            <div class="param-inputs-row">
              <div class="num-input-item">
                <label>TCP Offset (mm)</label>
                <input type="number" id="inp-tcp-offset" min="50" max="300" step="1" value="145" />
              </div>
              <div class="num-input-item">
                <label>Pre-Grasp (mm)</label>
                <input type="number" id="inp-pregrasp-dist" min="20" max="400" step="5" value="100" />
              </div>
            </div>
          </div>

          <!-- 7. SAVE GRASP ACTION -->
          <div class="property-section save-section">
            <button id="btn-save-grasp" class="btn btn-primary btn-save-grasp">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/>
                <polyline points="17 21 17 13 7 13 7 21"/>
                <polyline points="7 3 7 8 15 8"/>
              </svg>
              <span>Save Grasp Pose</span>
            </button>
          </div>
        </div>
      </div>
    `;

    this.bindEvents();
  }

  private updateValues() {
    if (!this.state) return;

    // Update collision & contact badges
    const badge = this.container.querySelector('#collision-badge');
    if (badge) {
      if (this.state.contactStatus === 'collision' || this.state.hasCollision) {
        badge.className = 'badge-status badge-collision';
        badge.textContent = 'Collision';
      } else if (this.state.contactStatus === 'contact') {
        badge.className = 'badge-status badge-contact';
        badge.textContent = 'Contact';
      } else {
        badge.className = 'badge-status badge-no-collision';
        badge.textContent = 'No collision';
      }
    }

    // Contact status card
    const statusText = this.container.querySelector('#contact-status-text');
    const messageText = this.container.querySelector('#contact-message-text');
    const measuredWidthLabel = this.container.querySelector('#label-measured-width');
    const fingersTouchedLabel = this.container.querySelector('#label-fingers-touched');

    if (statusText) {
      statusText.className = `contact-status-badge badge-contact-${this.state.contactStatus}`;
      const statusLabels: Record<string, string> = {
        none: 'No contact',
        contact: 'Contact Made',
        collision: 'Collision',
        missed: 'Missed',
        'too-wide': 'Part Too Wide'
      };
      statusText.textContent = statusLabels[this.state.contactStatus] || this.state.contactStatus;
    }

    if (messageText) {
      messageText.textContent = this.state.contactMessage || 'Ready';
    }

    if (measuredWidthLabel) {
      const w = this.state.measuredGraspWidth !== undefined
        ? this.state.measuredGraspWidth
        : this.state.openingWidth;
      measuredWidthLabel.textContent = `${w.toFixed(1)} mm`;
    }

    if (fingersTouchedLabel) {
      if (this.state.leftFingerContact && this.state.rightFingerContact) {
        fingersTouchedLabel.textContent = 'Both (Left & Right)';
      } else if (this.state.leftFingerContact) {
        fingersTouchedLabel.textContent = 'Left Finger';
      } else if (this.state.rightFingerContact) {
        fingersTouchedLabel.textContent = 'Right Finger';
      } else {
        fingersTouchedLabel.textContent = 'None';
      }
    }

    // Update test phase badge
    const phaseBadge = this.container.querySelector('#test-phase-badge');
    const testBtn = this.container.querySelector<HTMLButtonElement>('#btn-test-grasp');
    const stopBtn = this.container.querySelector<HTMLButtonElement>('#btn-stop-test');
    if (phaseBadge) {
      const phaseLabels: Record<string, string> = {
        idle: 'Idle',
        'pre-grasp': 'Pre-Grasp (Open)',
        approach: 'Approach (Detecting Collision)',
        close: 'Contact-Aware Close',
        lift: 'Lift (CAD Part Moving!)',
        transfer: 'Transfer',
        place: 'Place',
        release: 'Release',
        retract: 'Retract'
      };
      phaseBadge.textContent = phaseLabels[this.state.testPhase] || this.state.testPhase;
      phaseBadge.classList.toggle('active', this.state.isTesting);
    }
    if (testBtn) testBtn.disabled = this.state.isTesting;
    if (stopBtn) stopBtn.disabled = !this.state.isTesting;

    // Update Opening Slider
    const slider = this.container.querySelector<HTMLInputElement>('#slider-opening');
    const widthLabel = this.container.querySelector('#label-opening-width');

    if (slider && document.activeElement !== slider) {
      slider.value = this.state.openingWidth.toString();
    }
    if (widthLabel) {
      widthLabel.textContent = `${this.state.openingWidth.toFixed(1)} mm`;
    }

    // Update numeric pose fields
    const inpX = this.container.querySelector<HTMLInputElement>('#inp-pos-x');
    const inpY = this.container.querySelector<HTMLInputElement>('#inp-pos-y');
    const inpZ = this.container.querySelector<HTMLInputElement>('#inp-pos-z');
    const inpR = this.container.querySelector<HTMLInputElement>('#inp-rpy-r');
    const inpP = this.container.querySelector<HTMLInputElement>('#inp-rpy-p');
    const inpYaw = this.container.querySelector<HTMLInputElement>('#inp-rpy-y');

    if (inpX && document.activeElement !== inpX) inpX.value = this.state.pose.position.x.toFixed(2);
    if (inpY && document.activeElement !== inpY) inpY.value = this.state.pose.position.y.toFixed(2);
    if (inpZ && document.activeElement !== inpZ) inpZ.value = this.state.pose.position.z.toFixed(2);

    if (inpR && document.activeElement !== inpR) inpR.value = this.state.rpy.roll.toFixed(1);
    if (inpP && document.activeElement !== inpP) inpP.value = this.state.rpy.pitch.toFixed(1);
    if (inpYaw && document.activeElement !== inpYaw) inpYaw.value = this.state.rpy.yaw.toFixed(1);

    // Update read-only Gripper relative to part readout
    const relX = this.container.querySelector('#rel-pos-x');
    const relY = this.container.querySelector('#rel-pos-y');
    const relZ = this.container.querySelector('#rel-pos-z');
    const relR = this.container.querySelector('#rel-rpy-r');
    const relP = this.container.querySelector('#rel-rpy-p');
    const relYaw = this.container.querySelector('#rel-rpy-y');

    if (relX && this.state.relativePose) relX.textContent = this.state.relativePose.position.x.toFixed(2);
    if (relY && this.state.relativePose) relY.textContent = this.state.relativePose.position.y.toFixed(2);
    if (relZ && this.state.relativePose) relZ.textContent = this.state.relativePose.position.z.toFixed(2);

    if (relR && this.state.relativeRpy) relR.textContent = `${this.state.relativeRpy.roll.toFixed(1)}°`;
    if (relP && this.state.relativeRpy) relP.textContent = `${this.state.relativeRpy.pitch.toFixed(1)}°`;
    if (relYaw && this.state.relativeRpy) relYaw.textContent = `${this.state.relativeRpy.yaw.toFixed(1)}°`;

    // Update parameters
    const inpOffset = this.container.querySelector<HTMLInputElement>('#inp-tcp-offset');
    const inpPregrasp = this.container.querySelector<HTMLInputElement>('#inp-pregrasp-dist');
    const inpStandoff = this.container.querySelector<HTMLInputElement>('#inp-standoff');
    const inpDepth = this.container.querySelector<HTMLInputElement>('#inp-depth');

    if (inpOffset && document.activeElement !== inpOffset) inpOffset.value = this.state.tcpOffset.toString();
    if (inpPregrasp && document.activeElement !== inpPregrasp) inpPregrasp.value = this.state.preGraspDistance.toString();
    if (inpStandoff && document.activeElement !== inpStandoff) inpStandoff.value = this.state.standoff.toString();
    if (inpDepth && document.activeElement !== inpDepth) inpDepth.value = this.state.depth.toString();
  }

  private bindEvents() {
    // Contact Grip actions
    this.container.querySelector('#btn-close-contact')?.addEventListener('click', () => {
      this.events.onCloseUntilContact(this.tolerance);
    });

    this.container.querySelector('#btn-open-clear')?.addEventListener('click', () => {
      this.events.onOpenToClear();
    });

    this.container.querySelector('#btn-auto-place')?.addEventListener('click', () => {
      this.events.onAutoPlaceOnOuterWall();
    });

    // Contact parameters
    const inpStandoff = this.container.querySelector<HTMLInputElement>('#inp-standoff');
    inpStandoff?.addEventListener('change', (e) => {
      this.standoff = parseFloat((e.target as HTMLInputElement).value) || 0;
      this.events.onStandoffChange(this.standoff);
    });

    const inpDepth = this.container.querySelector<HTMLInputElement>('#inp-depth');
    inpDepth?.addEventListener('change', (e) => {
      this.depth = parseFloat((e.target as HTMLInputElement).value) || 15;
      this.events.onDepthChange(this.depth);
    });

    const inpTol = this.container.querySelector<HTMLInputElement>('#inp-tolerance');
    inpTol?.addEventListener('change', (e) => {
      this.tolerance = parseFloat((e.target as HTMLInputElement).value) || 0.1;
      this.events.onToleranceChange(this.tolerance);
    });

    // Test Grasp
    this.container.querySelector('#btn-test-grasp')?.addEventListener('click', () => {
      this.events.onTestGrasp();
    });

    this.container.querySelector('#btn-stop-test')?.addEventListener('click', () => {
      this.events.onStopTest();
    });

    // Alignment & Snapping
    this.container.querySelector('#btn-snap-point')?.addEventListener('click', () => {
      this.events.onSnapToClickedPoint();
    });

    this.container.querySelector('#btn-flip-approach')?.addEventListener('click', () => {
      this.events.onFlipApproach();
    });

    const spinBtns = this.container.querySelectorAll<HTMLButtonElement>('.spin-buttons button');
    spinBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        const deg = parseFloat(btn.dataset.spin || '0');
        this.events.onRotateApproach(deg);
      });
    });

    // Step size buttons
    const stepBtns = this.container.querySelectorAll<HTMLButtonElement>('.step-btn');
    stepBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        stepBtns.forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        this.linearStep = parseFloat(btn.dataset.linear || '5');
        this.angularStep = parseFloat(btn.dataset.angular || '15');
      });
    });

    // Translation Jog buttons
    const jogBtns = this.container.querySelectorAll<HTMLButtonElement>('.btn-jog');
    jogBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        const axis = btn.dataset.axis as 'x' | 'y' | 'z';
        const dir = parseFloat(btn.dataset.dir || '1');
        this.events.onStepTranslate(axis, dir * this.linearStep);
      });
    });

    // Rotation Jog buttons
    const rotBtns = this.container.querySelectorAll<HTMLButtonElement>('.btn-rot');
    rotBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        const axis = btn.dataset.axis as 'x' | 'y' | 'z';
        const dir = parseFloat(btn.dataset.dir || '1');
        this.events.onStepRotate(axis, dir * this.angularStep);
      });
    });

    // Numeric TCP Pose
    const handleNumeric = () => {
      const inpX = this.container.querySelector<HTMLInputElement>('#inp-pos-x');
      const inpY = this.container.querySelector<HTMLInputElement>('#inp-pos-y');
      const inpZ = this.container.querySelector<HTMLInputElement>('#inp-pos-z');
      const inpR = this.container.querySelector<HTMLInputElement>('#inp-rpy-r');
      const inpP = this.container.querySelector<HTMLInputElement>('#inp-rpy-p');
      const inpYaw = this.container.querySelector<HTMLInputElement>('#inp-rpy-y');

      if (!inpX || !inpY || !inpZ || !inpR || !inpP || !inpYaw) return;

      const pos = {
        x: parseFloat(inpX.value) || 0,
        y: parseFloat(inpY.value) || 0,
        z: parseFloat(inpZ.value) || 0
      };

      const rpy = {
        roll: parseFloat(inpR.value) || 0,
        pitch: parseFloat(inpP.value) || 0,
        yaw: parseFloat(inpYaw.value) || 0
      };

      this.events.onNumericPoseChange(pos, rpy);
    };

    ['#inp-pos-x', '#inp-pos-y', '#inp-pos-z', '#inp-rpy-r', '#inp-rpy-p', '#inp-rpy-y'].forEach(
      (sel) => {
        const input = this.container.querySelector<HTMLInputElement>(sel);
        input?.addEventListener('change', handleNumeric);
        input?.addEventListener('keydown', (e) => {
          if (e.key === 'Enter') handleNumeric();
        });
      }
    );

    this.container.querySelector('#btn-reset-pose')?.addEventListener('click', () => {
      this.events.onResetPose();
    });

    // Opening width slider
    const slider = this.container.querySelector<HTMLInputElement>('#slider-opening');
    slider?.addEventListener('input', (e) => {
      const val = parseFloat((e.target as HTMLInputElement).value);
      this.events.onOpeningChange(val);
    });

    // Quick presets for opening
    this.container.querySelector('#btn-quick-close')?.addEventListener('click', () => {
      this.events.onOpeningChange(0);
    });
    this.container.querySelector('#btn-quick-half')?.addEventListener('click', () => {
      this.events.onOpeningChange(42.5);
    });
    this.container.querySelector('#btn-quick-open')?.addEventListener('click', () => {
      this.events.onOpeningChange(85);
    });

    // TCP Offset & Pre-Grasp Distance
    const inpOffset = this.container.querySelector<HTMLInputElement>('#inp-tcp-offset');
    inpOffset?.addEventListener('change', (e) => {
      const val = parseFloat((e.target as HTMLInputElement).value);
      if (!isNaN(val)) this.events.onTcpOffsetChange(val);
    });

    const inpPregrasp = this.container.querySelector<HTMLInputElement>('#inp-pregrasp-dist');
    inpPregrasp?.addEventListener('change', (e) => {
      const val = parseFloat((e.target as HTMLInputElement).value);
      if (!isNaN(val)) this.events.onPreGraspDistanceChange(val);
    });

    // Save Grasp Pose
    this.container.querySelector('#btn-save-grasp')?.addEventListener('click', () => {
      this.events.onSaveGrasp();
    });
  }
}
