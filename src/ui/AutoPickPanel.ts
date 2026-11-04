/**
 * Left-sidebar panel for the automatic pick & place: shows where the user is in
 * Load STEP -> Mark two points -> Plan -> Preview -> Execute, and holds the actions.
 * State lives in AutoPickController; this class only renders it.
 */
export interface AutoPickPanelEvents {
  onCalculate: () => void;
  onClearPoints: () => void;
  onSelectGrasp: (index: number) => void;
  onPreviewHere: () => void;
  onPreviewRobot: () => void;
  onExecute: () => void;
}

export interface AutoPickView {
  partName: string | null;
  points: string[]; // marked points so far, at most two
  pointColors: string[];
  gripHint: { ok: boolean; text: string } | null; // what the two points mean for the gripper
  busy: boolean; // planning or a simulator run in progress
  playing: boolean; // viewport playback running
  status: { text: string; kind: 'info' | 'ok' | 'error' } | null;
  grasps: { title: string; detail: string }[];
  selected: number;
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
    const pointStep = (i: number) => {
      const text = view.points[i];
      const hint = view.points.length === i ? `Click the part where finger ${i + 1} should go` : `Finger ${i + 1}`;
      return step(!!text, String(i + 1), text ? `Finger ${i + 1}: ${escapeHtml(text)}` : hint, view.pointColors[i]);
    };
    const planned = view.grasps.length > 0;
    const canCalculate = !!view.partName && view.points.length === 2 && !view.busy;

    this.container.innerHTML = `
      <div class="sidebar-block auto-pick-block">
        <div class="block-header"><span class="block-title">AUTO PICK &amp; PLACE</span></div>
        ${step(!!view.partName, view.partName ? '✓' : '·', view.partName ? escapeHtml(view.partName) : 'Load a STEP file (Load CAD)')}
        ${view.partName ? pointStep(0) + pointStep(1) : ''}
        ${view.gripHint ? `<div id="auto-pick-grip-hint" class="auto-pick-hint ${view.gripHint.ok ? '' : 'is-bad'}">${escapeHtml(view.gripHint.text)}</div>` : ''}
        <div class="auto-pick-actions">
          <button id="auto-pick-calculate" class="btn btn-primary btn-sm" ${canCalculate ? '' : 'disabled'} title="Plan the motion that puts one finger on each marked point">Plan to Points</button>
          <button id="auto-pick-clear" class="btn btn-secondary btn-sm" ${view.points.length && !view.busy ? '' : 'disabled'} title="Remove the marked points">Clear Points</button>
        </div>
        ${view.status ? `<div id="auto-pick-status" class="auto-pick-status ${view.status.kind === 'error' ? 'is-error' : view.status.kind === 'ok' ? 'is-ok' : ''}">${escapeHtml(view.status.text)}</div>` : ''}
        ${
          planned
            ? `<div class="auto-pick-grasps">
                ${view.grasps
                  .map(
                    (g, i) => `
                    <button class="auto-pick-grasp ${i === view.selected ? 'is-selected' : ''}" data-index="${i}" ${view.busy || view.playing ? 'disabled' : ''}>
                      <span class="auto-pick-grasp-title">${escapeHtml(g.title)}</span>
                      <span class="auto-pick-grasp-detail">${escapeHtml(g.detail)}</span>
                    </button>`
                  )
                  .join('')}
              </div>
              <div class="auto-pick-actions">
                <button id="auto-pick-preview-here" class="btn btn-secondary btn-sm" ${view.busy ? 'disabled' : ''} title="Play the gripper and part motion in this view">${view.playing ? 'Stop Preview' : 'Preview Here'}</button>
                <button id="auto-pick-preview-robot" class="btn btn-secondary btn-sm" ${view.busy || view.playing ? 'disabled' : ''} title="Play the planned motion with the robot arm in a MuJoCo window, no physics">Preview Robot</button>
                <button id="auto-pick-execute" class="btn btn-primary btn-sm auto-pick-wide" ${view.busy || view.playing ? 'disabled' : ''} title="Run the pick &amp; place with full physics in MuJoCo">Execute Pick &amp; Place</button>
              </div>`
            : ''
        }
      </div>
    `;

    const on = (id: string, handler?: () => void) =>
      this.container.querySelector(`#${id}`)?.addEventListener('click', () => handler?.());
    on('auto-pick-calculate', this.events?.onCalculate);
    on('auto-pick-clear', this.events?.onClearPoints);
    on('auto-pick-preview-here', this.events?.onPreviewHere);
    on('auto-pick-preview-robot', this.events?.onPreviewRobot);
    on('auto-pick-execute', this.events?.onExecute);
    this.container.querySelectorAll<HTMLButtonElement>('.auto-pick-grasp').forEach((btn) => {
      btn.addEventListener('click', () => this.events?.onSelectGrasp(Number(btn.dataset.index)));
    });
  }
}
