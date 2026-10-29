import type { PartPoseState, EulerAngles } from '../types';

export interface PartPanelEvents {
  onNumericPoseChange: (pos: { x: number; y: number; z: number }, rpy: EulerAngles) => void;
  onStepTranslate: (axis: 'x' | 'y' | 'z', delta: number) => void;
  onStepRotate: (axis: 'x' | 'y' | 'z', delta: number) => void;
  onLockGripperToggle: (locked: boolean) => void;
  onResetPartPose: () => void;
  onBakePartOrigin: () => void;
  onTargetToggle: (target: 'part' | 'gripper') => void;
}

export class PartPanel {
  private container: HTMLElement;
  private events: PartPanelEvents;
  private state: PartPoseState | null = null;
  private activeTarget: 'part' | 'gripper' = 'gripper';

  private linearStep = 5; // mm
  private angularStep = 15; // deg

  constructor(container: HTMLElement, events: PartPanelEvents) {
    this.container = container;
    this.events = events;
    this.render();
  }

  public setPartState(state: PartPoseState) {
    this.state = state;
    this.updateValues();
  }

  public setActiveTarget(target: 'part' | 'gripper') {
    this.activeTarget = target;
    const btnPart = this.container.querySelector<HTMLButtonElement>('#seg-edit-part');
    const btnGripper = this.container.querySelector<HTMLButtonElement>('#seg-edit-gripper');
    if (btnPart && btnGripper) {
      btnPart.classList.toggle('active', target === 'part');
      btnGripper.classList.toggle('active', target === 'gripper');
    }
  }

  private render() {
    this.container.innerHTML = `
      <div class="sidebar-block part-panel-block">
        <div class="block-header">
          <div class="title-with-badge">
            <span class="block-title">PART POSE CONTROL</span>
          </div>
        </div>

        <div class="part-scrollable">
          <!-- 1. EDIT TARGET TOGGLE & LOCK GRIPPER -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M12 20h9"/>
                <path d="M16.5 3.5a2.121 2.121 0 0 1 3 3L7 19l-4 1 1-4L16.5 3.5z"/>
              </svg>
              <span>Gizmo Target</span>
            </div>

            <div class="edit-target-selector">
              <div class="segmented-control target-seg">
                <button id="seg-edit-part" class="seg-btn ${this.activeTarget === 'part' ? 'active' : ''}">Edit Part</button>
                <button id="seg-edit-gripper" class="seg-btn ${this.activeTarget === 'gripper' ? 'active' : ''}">Edit Gripper</button>
              </div>
            </div>

            <div class="lock-checkbox-row">
              <label class="checkbox-label" title="When ON, moving or rotating the part moves the gripper with it. When OFF (default), the part moves independently without moving the gripper.">
                <input type="checkbox" id="chk-lock-gripper" />
                <span class="checkbox-custom"></span>
                <span class="checkbox-text">Move gripper together with part</span>
              </label>
            </div>
          </div>

          <!-- 2. STEP JOGGING CONTROLS -->
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
              <span>Part Step Jogging</span>
            </div>

            <div class="step-sizes-selector">
              <span class="sub-label">Step:</span>
              <div class="segmented-control step-seg">
                <button class="seg-btn part-step-btn" data-linear="1" data-angular="1">1mm / 1°</button>
                <button class="seg-btn part-step-btn active" data-linear="5" data-angular="15">5mm / 15°</button>
                <button class="seg-btn part-step-btn" data-linear="10" data-angular="45">10mm / 45°</button>
                <button class="seg-btn part-step-btn" data-linear="25" data-angular="90">25mm / 90°</button>
              </div>
            </div>

            <div class="jog-grid">
              <!-- Translation Jog -->
              <div class="jog-group">
                <span class="axis-header">Position (mm)</span>
                <div class="jog-row">
                  <span class="axis-tag axis-x">X</span>
                  <button class="btn-jog part-btn-jog" data-axis="x" data-dir="-1">-</button>
                  <button class="btn-jog part-btn-jog" data-axis="x" data-dir="1">+</button>
                </div>
                <div class="jog-row">
                  <span class="axis-tag axis-y">Y</span>
                  <button class="btn-jog part-btn-jog" data-axis="y" data-dir="-1">-</button>
                  <button class="btn-jog part-btn-jog" data-axis="y" data-dir="1">+</button>
                </div>
                <div class="jog-row">
                  <span class="axis-tag axis-z">Z</span>
                  <button class="btn-jog part-btn-jog" data-axis="z" data-dir="-1">-</button>
                  <button class="btn-jog part-btn-jog" data-axis="z" data-dir="1">+</button>
                </div>
              </div>

              <!-- Rotation Jog -->
              <div class="jog-group">
                <span class="axis-header">Rotation (deg)</span>
                <div class="jog-row">
                  <span class="axis-tag">Rx</span>
                  <button class="btn-rot part-btn-rot" data-axis="x" data-dir="-1">-</button>
                  <button class="btn-rot part-btn-rot" data-axis="x" data-dir="1">+</button>
                </div>
                <div class="jog-row">
                  <span class="axis-tag">Ry</span>
                  <button class="btn-rot part-btn-rot" data-axis="y" data-dir="-1">-</button>
                  <button class="btn-rot part-btn-rot" data-axis="y" data-dir="1">+</button>
                </div>
                <div class="jog-row">
                  <span class="axis-tag">Rz</span>
                  <button class="btn-rot part-btn-rot" data-axis="z" data-dir="-1">-</button>
                  <button class="btn-rot part-btn-rot" data-axis="z" data-dir="1">+</button>
                </div>
              </div>
            </div>
          </div>

          <!-- 3. NUMERIC POSE (TWO-WAY BINDING) -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="4" y="4" width="16" height="16" rx="2" ry="2"/>
                <line x1="9" y1="9" x2="15" y2="15"/>
                <line x1="15" y1="9" x2="9" y2="15"/>
              </svg>
              <span>Part Pose (World Frame)</span>
            </div>

            <div class="numeric-inputs-grid">
              <div class="num-input-item">
                <label>X (mm)</label>
                <input type="number" id="inp-part-x" step="0.5" value="0.00" />
              </div>
              <div class="num-input-item">
                <label>Y (mm)</label>
                <input type="number" id="inp-part-y" step="0.5" value="0.00" />
              </div>
              <div class="num-input-item">
                <label>Z (mm)</label>
                <input type="number" id="inp-part-z" step="0.5" value="0.00" />
              </div>
              <div class="num-input-item">
                <label>Roll (°)</label>
                <input type="number" id="inp-part-r" step="1" value="0.0" />
              </div>
              <div class="num-input-item">
                <label>Pitch (°)</label>
                <input type="number" id="inp-part-p" step="1" value="0.0" />
              </div>
              <div class="num-input-item">
                <label>Yaw (°)</label>
                <input type="number" id="inp-part-y-rot" step="1" value="0.0" />
              </div>
            </div>

            <div class="part-actions-column">
              <button id="btn-reset-part-pose" class="btn btn-secondary btn-sm btn-full">
                Reset Part Pose
              </button>
              <button id="btn-bake-part-origin" class="btn btn-outline btn-sm btn-full" title="Bakes current part orientation into geometry. Saved grasps will be invalidated.">
                Set Current Orientation as Part Origin
              </button>
            </div>
          </div>
        </div>
      </div>
    `;

    this.bindEvents();
  }

  private updateValues() {
    if (!this.state) return;

    const inpX = this.container.querySelector<HTMLInputElement>('#inp-part-x');
    const inpY = this.container.querySelector<HTMLInputElement>('#inp-part-y');
    const inpZ = this.container.querySelector<HTMLInputElement>('#inp-part-z');
    const inpR = this.container.querySelector<HTMLInputElement>('#inp-part-r');
    const inpP = this.container.querySelector<HTMLInputElement>('#inp-part-p');
    const inpYaw = this.container.querySelector<HTMLInputElement>('#inp-part-y-rot');
    const chkLock = this.container.querySelector<HTMLInputElement>('#chk-lock-gripper');

    if (inpX && document.activeElement !== inpX) inpX.value = this.state.pose.position.x.toFixed(2);
    if (inpY && document.activeElement !== inpY) inpY.value = this.state.pose.position.y.toFixed(2);
    if (inpZ && document.activeElement !== inpZ) inpZ.value = this.state.pose.position.z.toFixed(2);

    if (inpR && document.activeElement !== inpR) inpR.value = this.state.rpy.roll.toFixed(1);
    if (inpP && document.activeElement !== inpP) inpP.value = this.state.rpy.pitch.toFixed(1);
    if (inpYaw && document.activeElement !== inpYaw) inpYaw.value = this.state.rpy.yaw.toFixed(1);

    if (chkLock) {
      chkLock.checked = this.state.moveGripperTogetherWithPart ?? this.state.lockGripperToPart ?? false;
    }
  }

  private bindEvents() {
    // 1. Edit Target Toggle
    const btnPart = this.container.querySelector<HTMLButtonElement>('#seg-edit-part');
    const btnGripper = this.container.querySelector<HTMLButtonElement>('#seg-edit-gripper');

    btnPart?.addEventListener('click', () => {
      this.setActiveTarget('part');
      this.events.onTargetToggle('part');
    });

    btnGripper?.addEventListener('click', () => {
      this.setActiveTarget('gripper');
      this.events.onTargetToggle('gripper');
    });

    // 2. Lock Gripper Checkbox
    const chkLock = this.container.querySelector<HTMLInputElement>('#chk-lock-gripper');
    chkLock?.addEventListener('change', (e) => {
      this.events.onLockGripperToggle((e.target as HTMLInputElement).checked);
    });

    // 3. Step buttons
    const stepBtns = this.container.querySelectorAll<HTMLButtonElement>('.part-step-btn');
    stepBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        stepBtns.forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        this.linearStep = parseFloat(btn.dataset.linear || '5');
        this.angularStep = parseFloat(btn.dataset.angular || '15');
      });
    });

    // 4. Jog Translation
    const jogBtns = this.container.querySelectorAll<HTMLButtonElement>('.part-btn-jog');
    jogBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        const axis = btn.dataset.axis as 'x' | 'y' | 'z';
        const dir = parseFloat(btn.dataset.dir || '1');
        this.events.onStepTranslate(axis, dir * this.linearStep);
      });
    });

    // 5. Jog Rotation
    const rotBtns = this.container.querySelectorAll<HTMLButtonElement>('.part-btn-rot');
    rotBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        const axis = btn.dataset.axis as 'x' | 'y' | 'z';
        const dir = parseFloat(btn.dataset.dir || '1');
        this.events.onStepRotate(axis, dir * this.angularStep);
      });
    });

    // 6. Numeric inputs
    const handleNumeric = () => {
      const inpX = this.container.querySelector<HTMLInputElement>('#inp-part-x');
      const inpY = this.container.querySelector<HTMLInputElement>('#inp-part-y');
      const inpZ = this.container.querySelector<HTMLInputElement>('#inp-part-z');
      const inpR = this.container.querySelector<HTMLInputElement>('#inp-part-r');
      const inpP = this.container.querySelector<HTMLInputElement>('#inp-part-p');
      const inpYaw = this.container.querySelector<HTMLInputElement>('#inp-part-y-rot');

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

    ['#inp-part-x', '#inp-part-y', '#inp-part-z', '#inp-part-r', '#inp-part-p', '#inp-part-y-rot'].forEach(
      (sel) => {
        const input = this.container.querySelector<HTMLInputElement>(sel);
        input?.addEventListener('change', handleNumeric);
        input?.addEventListener('keydown', (e) => {
          if (e.key === 'Enter') handleNumeric();
        });
      }
    );

    // 7. Reset Part Pose
    this.container.querySelector('#btn-reset-part-pose')?.addEventListener('click', () => {
      this.events.onResetPartPose();
    });

    // 8. Bake Part Origin
    this.container.querySelector('#btn-bake-part-origin')?.addEventListener('click', () => {
      this.events.onBakePartOrigin();
    });
  }
}
