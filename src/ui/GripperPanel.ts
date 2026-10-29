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
}

export class GripperPanel {
  private container: HTMLElement;
  private events: GripperPanelEvents;
  private state: GripperState | null = null;
  private canSnap = false;

  // Selected step sizes
  private linearStep = 5; // mm
  private angularStep = 15; // deg

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
          <!-- 1. TEST GRASP CONTROLS & STATUS -->
          <div class="property-section test-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <polygon points="5 3 19 12 5 21 5 3"/>
              </svg>
              <span>Pick & Place Motion</span>
              <span id="test-phase-badge" class="test-phase-badge">Idle</span>
            </div>

            <div class="test-buttons-row">
              <button id="btn-test-grasp" class="btn btn-primary btn-test" title="Run Pick & Place sequence: Approach, Clamp on surface, Lift CAD model, Place and Release">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <polygon points="5 3 19 12 5 21 5 3"/>
                </svg>
                <span>Run Pick & Place</span>
              </button>
              <button id="btn-stop-test" class="btn btn-secondary btn-test" disabled>
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <rect x="6" y="6" width="12" height="12"/>
                </svg>
                <span>Stop</span>
              </button>
            </div>
          </div>

          <!-- 2. QUICK ALIGNMENT & SNAPPING -->
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

          <!-- 3. STEP JOGGING CONTROLS -->
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

          <!-- 4. NUMERIC POSE (TWO-WAY BINDING) -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="4" y="4" width="16" height="16" rx="2" ry="2"/>
                <line x1="9" y1="9" x2="15" y2="15"/>
                <line x1="15" y1="9" x2="9" y2="15"/>
              </svg>
              <span>TCP Pose (Part Frame)</span>
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

            <button id="btn-reset-pose" class="btn btn-secondary btn-sm btn-full">
              Reset Gripper Pose
            </button>
          </div>

          <!-- 5. FINGER OPEN / CLOSE & TCP CONFIG -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M7 11V7a5 5 0 0 1 10 0v4"/>
                <rect x="3" y="11" width="18" height="11" rx="2" ry="2"/>
              </svg>
              <span>Fingers & Dimensions</span>
            </div>

            <!-- Open / Close Slider -->
            <div class="slider-control-group">
              <div class="slider-header">
                <span class="sub-label">Opening Width:</span>
                <div class="slider-val-wrap">
                  <span id="label-contact-block" class="contact-block-badge" style="display:none"></span>
                  <span id="label-opening-width" class="slider-value mono">85.0 mm</span>
                </div>
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

          <!-- 6. SAVE GRASP ACTION -->
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

    // Update collision badge
    const badge = this.container.querySelector('#collision-badge');
    if (badge) {
      if (this.state.hasCollision) {
        badge.className = 'badge-status badge-collision';
        badge.textContent = 'Collision';
      } else {
        badge.className = 'badge-status badge-no-collision';
        badge.textContent = 'No collision';
      }
    }

    // Update test phase badge
    const phaseBadge = this.container.querySelector('#test-phase-badge');
    const testBtn = this.container.querySelector<HTMLButtonElement>('#btn-test-grasp');
    const stopBtn = this.container.querySelector<HTMLButtonElement>('#btn-stop-test');
    if (phaseBadge) {
      const phaseLabels: Record<string, string> = {
        idle: 'Idle',
        'pre-grasp': 'Pre-Grasp (Standoff)',
        approach: 'Approach (Blocked at Surface)',
        close: 'Clamp (Contact Boundary)',
        lift: 'Lift (Model Picked!)',
        transfer: 'Transfer (Carrying Part)',
        place: 'Place (Lowering to Table)',
        release: 'Release (Fingers Open)',
        retract: 'Retract'
      };
      phaseBadge.textContent = phaseLabels[this.state.testPhase] || this.state.testPhase;
      phaseBadge.classList.toggle('active', this.state.isTesting);
    }
    if (testBtn) testBtn.disabled = this.state.isTesting;
    if (stopBtn) stopBtn.disabled = !this.state.isTesting;

    // Update Opening Slider & Contact Block Badge
    const slider = this.container.querySelector<HTMLInputElement>('#slider-opening');
    const widthLabel = this.container.querySelector('#label-opening-width');
    const contactBlockBadge = this.container.querySelector<HTMLElement>('#label-contact-block');

    if (contactBlockBadge) {
      if (this.state.contactWidth && this.state.contactWidth > 0) {
        contactBlockBadge.style.display = 'inline-block';
        contactBlockBadge.textContent = `Blocked at ${this.state.contactWidth.toFixed(1)}mm`;
      } else {
        contactBlockBadge.style.display = 'none';
      }
    }
    if (slider && document.activeElement !== slider) {
      slider.value = this.state.openingWidth.toString();
    }
    if (widthLabel) {
      widthLabel.textContent = `${this.state.openingWidth.toFixed(1)} mm`;
    }

    // Update Numeric Inputs (do not overwrite if currently focused by user)
    const setIfUnfocused = (id: string, val: number) => {
      const el = this.container.querySelector<HTMLInputElement>(id);
      if (el && document.activeElement !== el) {
        el.value = val.toString();
      }
    };

    setIfUnfocused('#inp-pos-x', this.state.pose.position.x);
    setIfUnfocused('#inp-pos-y', this.state.pose.position.y);
    setIfUnfocused('#inp-pos-z', this.state.pose.position.z);
    setIfUnfocused('#inp-rpy-r', this.state.rpy.roll);
    setIfUnfocused('#inp-rpy-p', this.state.rpy.pitch);
    setIfUnfocused('#inp-rpy-y', this.state.rpy.yaw);
    setIfUnfocused('#inp-tcp-offset', this.state.tcpOffset);
    setIfUnfocused('#inp-pregrasp-dist', this.state.preGraspDistance);
  }

  private bindEvents() {
    // Snap & Flip
    this.container.querySelector('#btn-snap-point')?.addEventListener('click', () => {
      this.events.onSnapToClickedPoint();
    });
    this.container.querySelector('#btn-flip-approach')?.addEventListener('click', () => {
      this.events.onFlipApproach();
    });

    // Spin around approach
    this.container.querySelectorAll<HTMLButtonElement>('[data-spin]').forEach((btn) => {
      btn.addEventListener('click', () => {
        const deg = parseFloat(btn.dataset.spin || '0');
        if (deg !== 0) this.events.onRotateApproach(deg);
      });
    });

    // Step size selector
    this.container.querySelectorAll<HTMLButtonElement>('.step-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        this.container.querySelectorAll('.step-btn').forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        this.linearStep = parseFloat(btn.dataset.linear || '5');
        this.angularStep = parseFloat(btn.dataset.angular || '15');
      });
    });

    // Jog Translation
    this.container.querySelectorAll<HTMLButtonElement>('.btn-jog').forEach((btn) => {
      btn.addEventListener('click', () => {
        const axis = btn.dataset.axis as 'x' | 'y' | 'z';
        const dir = parseFloat(btn.dataset.dir || '1');
        this.events.onStepTranslate(axis, dir * this.linearStep);
      });
    });

    // Jog Rotation
    this.container.querySelectorAll<HTMLButtonElement>('.btn-rot').forEach((btn) => {
      btn.addEventListener('click', () => {
        const axis = btn.dataset.axis as 'x' | 'y' | 'z';
        const dir = parseFloat(btn.dataset.dir || '1');
        this.events.onStepRotate(axis, dir * this.angularStep);
      });
    });

    // Numeric inputs two-way binding
    const handleNumericSubmit = () => {
      const x = parseFloat((this.container.querySelector('#inp-pos-x') as HTMLInputElement).value) || 0;
      const y = parseFloat((this.container.querySelector('#inp-pos-y') as HTMLInputElement).value) || 0;
      const z = parseFloat((this.container.querySelector('#inp-pos-z') as HTMLInputElement).value) || 0;
      const roll = parseFloat((this.container.querySelector('#inp-rpy-r') as HTMLInputElement).value) || 0;
      const pitch = parseFloat((this.container.querySelector('#inp-rpy-p') as HTMLInputElement).value) || 0;
      const yaw = parseFloat((this.container.querySelector('#inp-rpy-y') as HTMLInputElement).value) || 0;

      this.events.onNumericPoseChange({ x, y, z }, { roll, pitch, yaw });
    };

    const numInputIds = [
      '#inp-pos-x',
      '#inp-pos-y',
      '#inp-pos-z',
      '#inp-rpy-r',
      '#inp-rpy-p',
      '#inp-rpy-y'
    ];
    numInputIds.forEach((id) => {
      const el = this.container.querySelector<HTMLInputElement>(id);
      if (el) {
        el.addEventListener('change', handleNumericSubmit);
        el.addEventListener('keydown', (e) => {
          if (e.key === 'Enter') handleNumericSubmit();
        });
      }
    });

    // Reset Pose
    this.container.querySelector('#btn-reset-pose')?.addEventListener('click', () => {
      this.events.onResetPose();
    });

    // Opening width slider & quick buttons
    const slider = this.container.querySelector<HTMLInputElement>('#slider-opening');
    if (slider) {
      slider.addEventListener('input', () => {
        const val = parseFloat(slider.value);
        this.events.onOpeningChange(val);
      });
    }

    this.container.querySelector('#btn-quick-close')?.addEventListener('click', () => {
      this.events.onOpeningChange(0);
    });
    this.container.querySelector('#btn-quick-half')?.addEventListener('click', () => {
      this.events.onOpeningChange(42.5);
    });
    this.container.querySelector('#btn-quick-open')?.addEventListener('click', () => {
      this.events.onOpeningChange(85);
    });

    // TCP Offset & Pre-Grasp inputs
    const tcpOffsetInp = this.container.querySelector<HTMLInputElement>('#inp-tcp-offset');
    tcpOffsetInp?.addEventListener('change', () => {
      const val = parseFloat(tcpOffsetInp.value) || 145;
      this.events.onTcpOffsetChange(val);
    });

    const preGraspInp = this.container.querySelector<HTMLInputElement>('#inp-pregrasp-dist');
    preGraspInp?.addEventListener('change', () => {
      const val = parseFloat(preGraspInp.value) || 100;
      this.events.onPreGraspDistanceChange(val);
    });

    // Test Grasp
    this.container.querySelector('#btn-test-grasp')?.addEventListener('click', () => {
      this.events.onTestGrasp();
    });
    this.container.querySelector('#btn-stop-test')?.addEventListener('click', () => {
      this.events.onStopTest();
    });

    // Save Grasp
    this.container.querySelector('#btn-save-grasp')?.addEventListener('click', () => {
      this.events.onSaveGrasp();
    });
  }
}
