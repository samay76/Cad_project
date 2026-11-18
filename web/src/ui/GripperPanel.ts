import type { GripperState } from '../types';

export interface GripperPanelEvents {
  onStepTranslate: (axis: 'x' | 'y' | 'z', delta: number) => void;
  onStepRotate: (axis: 'x' | 'y' | 'z', delta: number) => void;
}

export class GripperPanel {
  private container: HTMLElement;
  private events: GripperPanelEvents;
  private state: GripperState | null = null;

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

  private render() {
    this.container.innerHTML = `
      <div class="sidebar-block gripper-panel-block">
        <div class="block-header">
          <div class="title-with-badge">
            <span class="block-title">GRIPPER</span>
            <span id="collision-badge" class="badge-status badge-no-collision">No collision</span>
          </div>
        </div>

        <div class="gripper-scrollable">
          <!-- STEP JOGGING: moves the gripper; Plan to Points then plans to exactly this pose -->
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

  }

  private bindEvents() {
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
  }
}
