import type { SavedGrasp } from '../types';
import { GRIPPERS } from '../config';

export interface GraspListEvents {
  onRestoreGrasp: (grasp: SavedGrasp) => void;
  onRenameGrasp: (id: string, newName: string) => void;
  onDeleteGrasp: (id: string) => void;
  onExportJSON: () => void;
  onImportJSON: (importedGrasps: SavedGrasp[]) => void;
  onSimulateGrasp: (grasp: SavedGrasp) => void;
  onToggleRecord: () => void;
}

export class GraspListView {
  private container: HTMLElement;
  private events: GraspListEvents;
  private grasps: SavedGrasp[] = [];
  private selectedGraspId: string | null = null;
  private importFileInput!: HTMLInputElement;
  private recording = false;
  private recordedSamples = 0;

  constructor(container: HTMLElement, events: GraspListEvents) {
    this.container = container;
    this.events = events;
    this.render();
  }

  public setGrasps(grasps: SavedGrasp[]) {
    this.grasps = grasps;
    this.render();
  }

  /** Updates the Record button in place (no re-render, so it can be called while sampling). */
  public setRecording(active: boolean, samples = 0) {
    this.recording = active;
    this.recordedSamples = samples;
    const btn = this.container.querySelector<HTMLButtonElement>('#btn-record-motion');
    if (!btn) return;
    btn.classList.toggle('is-recording', active);
    btn.title = active
      ? 'Stop recording and save the motion as a grasp'
      : 'Record how the gripper moves to the part (move it, open/close the fingers), then press again to save';
    btn.querySelector('.rec-label')!.textContent = active
      ? `Stop & save recording (${samples} ${samples === 1 ? 'pose' : 'poses'})`
      : 'Record gripper motion';
  }

  public setSelectedGraspId(id: string | null) {
    this.selectedGraspId = id;
    this.updateSelectionDOM();
  }

  private render() {
    this.container.innerHTML = `
      <div class="sidebar-block grasp-list-block">
        <div class="block-header">
          <div class="title-with-badge">
            <span class="block-title">SAVED GRASPS</span>
            <span class="count-badge">${this.grasps.length}</span>
          </div>
        </div>

        <div class="grasps-scrollable-container">
          ${this.renderGraspsList()}
        </div>

        <div class="grasps-footer-actions">
          <button id="btn-record-motion" class="btn btn-secondary btn-sm btn-record ${this.recording ? 'is-recording' : ''}">
            <span class="rec-dot" aria-hidden="true"></span>
            <span class="rec-label">${this.recording ? `Stop &amp; save recording (${this.recordedSamples} ${this.recordedSamples === 1 ? 'pose' : 'poses'})` : 'Record gripper motion'}</span>
          </button>
          <button id="btn-simulate-mujoco" class="btn btn-primary btn-sm btn-simulate" ${this.grasps.length === 0 ? 'disabled' : ''} title="Run the selected grasp as a pick &amp; place in MuJoCo">
            <svg width="13" height="13" viewBox="0 0 24 24" fill="currentColor" stroke="none">
              <polygon points="6 4 20 12 6 20 6 4"/>
            </svg>
            <span>Simulate in MuJoCo</span>
          </button>
          <input type="file" id="import-grasps-input" class="hidden-file-input" accept=".json,application/json" />
          <button id="btn-export-grasps" class="btn btn-secondary btn-sm" ${this.grasps.length === 0 ? 'disabled' : ''} title="Export grasps to JSON">
            <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
              <polyline points="7 10 12 15 17 10"/>
              <line x1="12" y1="15" x2="12" y2="3"/>
            </svg>
            <span>Export JSON</span>
          </button>
          <button id="btn-import-grasps" class="btn btn-secondary btn-sm" title="Import grasps from JSON">
            <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
              <polyline points="17 8 12 3 7 8"/>
              <line x1="12" y1="3" x2="12" y2="15"/>
            </svg>
            <span>Import JSON</span>
          </button>
        </div>
      </div>
    `;

    this.importFileInput = this.container.querySelector('#import-grasps-input')!;
    this.bindEvents();
    this.setRecording(this.recording, this.recordedSamples);
  }

  private renderGraspsList(): string {
    if (this.grasps.length === 0) {
      return `
        <div class="empty-grasps-state">
          <div class="empty-icon">🤖</div>
          <p class="empty-text">No grasps saved yet</p>
          <p class="empty-subtext">Position the gripper and click "Save Grasp Pose"</p>
        </div>
      `;
    }

    return this.grasps
      .map((g) => {
        const isSelected = g.id === this.selectedGraspId;
        const pos = g.tcpPose.position;
        const coordsStr = `X: ${pos.x.toFixed(1)}, Y: ${pos.y.toFixed(1)}, Z: ${pos.z.toFixed(1)}`;
        const widthVal = g.measuredGraspWidth !== undefined ? g.measuredGraspWidth : g.openingWidth;
        const widthBadge = `<span class="grasp-width-badge mono">${widthVal.toFixed(1)}mm</span>`;
        const recBadge = g.trajectory?.length
          ? `<span class="grasp-rec-badge" title="Recorded motion: ${g.trajectory.length} poses, ${
              g.gripMode === 'inside' ? 'grips by opening the fingers (inside)' : 'grips by closing the fingers (outside)'
            }">REC · ${g.gripMode === 'inside' ? 'inside' : 'outside'}</span>`
          : '';

        return `
          <div class="grasp-item ${isSelected ? 'is-selected' : ''}" data-id="${g.id}">
            <div class="grasp-item-info">
              <div class="grasp-item-title-row">
                <span class="grasp-name" title="${g.name}">${g.name}</span>
                ${widthBadge}
              </div>
              <div class="grasp-coords mono">${coordsStr}</div>
              <div class="grasp-rec-row">
                <span class="grasp-gripper-badge" title="Taught with the ${GRIPPERS[g.gripperId ?? '2f85'].label}">${GRIPPERS[g.gripperId ?? '2f85'].label.replace('Robotiq ', '')}</span>
                ${recBadge}
              </div>
            </div>

            <div class="grasp-item-actions">
              <button class="btn-item-action btn-simulate-item" title="Simulate this grasp in MuJoCo" data-id="${g.id}">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="currentColor" stroke="none">
                  <polygon points="6 4 20 12 6 20 6 4"/>
                </svg>
              </button>
              <button class="btn-item-action btn-rename" title="Rename Grasp" data-id="${g.id}">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="M12 20h9"/>
                  <path d="M16.5 3.5a2.121 2.121 0 0 1 3 3L7 19l-4 1 1-4L16.5 3.5z"/>
                </svg>
              </button>
              <button class="btn-item-action btn-delete" title="Delete Grasp" data-id="${g.id}">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <polyline points="3 6 5 6 21 6"/>
                  <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>
                </svg>
              </button>
            </div>
          </div>
        `;
      })
      .join('');
  }

  private bindEvents() {
    // Restore grasp on click
    this.container.querySelectorAll<HTMLElement>('.grasp-item').forEach((item) => {
      item.addEventListener('click', (e) => {
        if ((e.target as HTMLElement).closest('.btn-item-action')) {
          return;
        }
        const id = item.dataset.id;
        const grasp = this.grasps.find((g) => g.id === id);
        if (grasp) {
          this.setSelectedGraspId(grasp.id);
          this.events.onRestoreGrasp(grasp);
        }
      });
    });

    // Rename Grasp
    this.container.querySelectorAll<HTMLButtonElement>('.btn-rename').forEach((btn) => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const id = btn.dataset.id;
        const grasp = this.grasps.find((g) => g.id === id);
        if (grasp) {
          const newName = prompt('Rename Grasp Pose:', grasp.name);
          if (newName && newName.trim()) {
            this.events.onRenameGrasp(grasp.id, newName.trim());
          }
        }
      });
    });

    // Delete Grasp
    this.container.querySelectorAll<HTMLButtonElement>('.btn-delete').forEach((btn) => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const id = btn.dataset.id;
        if (id && confirm('Delete this saved grasp pose?')) {
          this.events.onDeleteGrasp(id);
        }
      });
    });

    // Simulate a specific grasp in MuJoCo
    this.container.querySelectorAll<HTMLButtonElement>('.btn-simulate-item').forEach((btn) => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const grasp = this.grasps.find((g) => g.id === btn.dataset.id);
        if (grasp) {
          this.setSelectedGraspId(grasp.id);
          this.events.onSimulateGrasp(grasp);
        }
      });
    });

    // Simulate the selected grasp (or the most recent one if none is selected)
    this.container.querySelector('#btn-simulate-mujoco')?.addEventListener('click', () => {
      const grasp =
        this.grasps.find((g) => g.id === this.selectedGraspId) ?? this.grasps[this.grasps.length - 1];
      if (grasp) {
        this.setSelectedGraspId(grasp.id);
        this.events.onSimulateGrasp(grasp);
      }
    });

    // Record gripper motion
    this.container.querySelector('#btn-record-motion')?.addEventListener('click', () => {
      this.events.onToggleRecord();
    });

    // Export JSON
    this.container.querySelector('#btn-export-grasps')?.addEventListener('click', () => {
      this.events.onExportJSON();
    });

    // Import JSON
    const importBtn = this.container.querySelector('#btn-import-grasps');
    if (importBtn && this.importFileInput) {
      importBtn.addEventListener('click', () => {
        this.importFileInput.value = '';
        this.importFileInput.click();
      });

      this.importFileInput.addEventListener('change', async () => {
        if (this.importFileInput.files && this.importFileInput.files.length > 0) {
          const file = this.importFileInput.files[0];
          try {
            const text = await file.text();
            const data = JSON.parse(text);
            const imported: SavedGrasp[] = Array.isArray(data) ? data : [data];
            this.events.onImportJSON(imported);
          } catch (err: any) {
            alert(`Failed to import JSON: ${err?.message || 'Invalid format'}`);
          }
        }
      });
    }
  }

  private updateSelectionDOM() {
    this.container.querySelectorAll<HTMLElement>('.grasp-item').forEach((item) => {
      item.classList.toggle('is-selected', item.dataset.id === this.selectedGraspId);
    });
  }
}
