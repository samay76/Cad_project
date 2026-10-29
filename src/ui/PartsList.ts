import type { PartInfo } from '../types';

export interface ViewControlEvents {
  onResetView: () => void;
  onFitToModel: () => void;
  onToggleWireframe: () => boolean;
  onViewPreset: (preset: 'iso' | 'top' | 'front' | 'right') => void;
  onPartSelect: (index: number) => void;
}

export class PartsListView {
  private container: HTMLElement;
  private events: ViewControlEvents;
  private parts: PartInfo[] = [];
  private selectedPartIndex: number | null = null;
  private filterQuery = '';
  private isWireframeActive = false;

  constructor(container: HTMLElement, events: ViewControlEvents) {
    this.container = container;
    this.events = events;
    this.render();
  }

  public setParts(parts: PartInfo[]) {
    this.parts = parts;
    this.selectedPartIndex = null;
    this.filterQuery = '';
    this.render();
  }

  public setSelectedPart(index: number | null) {
    this.selectedPartIndex = index;
    this.updateSelectionDOM();
  }

  public setWireframeState(isActive: boolean) {
    this.isWireframeActive = isActive;
    const btn = this.container.querySelector<HTMLButtonElement>('#btn-wireframe');
    if (btn) {
      btn.classList.toggle('active', isActive);
      const label = btn.querySelector('.btn-label');
      if (label) label.textContent = isActive ? 'Wireframe: ON' : 'Wireframe: OFF';
    }
  }

  private render() {
    this.container.innerHTML = `
      <div class="sidebar-block view-controls-block">
        <div class="block-header">
          <span class="block-title">VIEW CONTROLS</span>
        </div>
        <div class="view-buttons-grid">
          <button id="btn-fit" class="btn btn-secondary btn-view" title="Fit Camera to Model">
            <svg class="btn-icon" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M8 3H5a2 2 0 0 0-2 2v3m18 0V5a2 2 0 0 0-2-2h-3m0 18h3a2 2 0 0 0 2-2v-3M3 16v3a2 2 0 0 0 2 2h3"/>
            </svg>
            <span>Fit Model</span>
          </button>
          <button id="btn-reset" class="btn btn-secondary btn-view" title="Reset View Angle">
            <svg class="btn-icon" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8"/>
              <path d="M3 3v5h5"/>
            </svg>
            <span>Reset View</span>
          </button>
        </div>

        <div class="view-preset-group">
          <div class="preset-label">Camera Angle</div>
          <div class="segmented-control">
            <button class="seg-btn" data-preset="iso" title="Isometric 3D">ISO</button>
            <button class="seg-btn" data-preset="top" title="Top View (Z-axis)">TOP</button>
            <button class="seg-btn" data-preset="front" title="Front View (-Y)">FRT</button>
            <button class="seg-btn" data-preset="right" title="Right View (+X)">RGT</button>
          </div>
        </div>

        <button id="btn-wireframe" class="btn btn-secondary btn-toggle ${this.isWireframeActive ? 'active' : ''}">
          <svg class="btn-icon" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <polygon points="12 2 2 7 12 12 22 7 12 2"/>
            <polyline points="2 17 12 22 22 17"/>
            <polyline points="2 12 12 17 22 12"/>
          </svg>
          <span class="btn-label">${this.isWireframeActive ? 'Wireframe: ON' : 'Wireframe: OFF'}</span>
        </button>
      </div>

      <div class="sidebar-block parts-list-block">
        <div class="block-header">
          <div class="title-with-badge">
            <span class="block-title">PARTS LIST</span>
            <span class="count-badge">${this.parts.length}</span>
          </div>
        </div>

        ${
          this.parts.length > 4
            ? `
          <div class="search-box">
            <svg class="search-icon" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <circle cx="11" cy="11" r="8"/>
              <line x1="21" y1="21" x2="16.65" y2="16.65"/>
            </svg>
            <input type="text" id="parts-filter" placeholder="Filter parts..." value="${this.filterQuery}" />
          </div>
        `
            : ''
        }

        <div class="parts-scrollable-container">
          ${this.renderPartsList()}
        </div>
      </div>
    `;

    this.bindEvents();
  }

  private renderPartsList(): string {
    if (this.parts.length === 0) {
      return `
        <div class="empty-parts-state">
          <div class="empty-icon">📂</div>
          <p class="empty-text">No CAD model loaded</p>
          <p class="empty-subtext">Click "Load CAD File" or drag & drop a file to view parts</p>
        </div>
      `;
    }

    const filtered = this.filterQuery
      ? this.parts.filter((p) => p.name.toLowerCase().includes(this.filterQuery.toLowerCase()))
      : this.parts;

    if (filtered.length === 0) {
      return `<div class="no-filter-match">No parts matching "${this.filterQuery}"</div>`;
    }

    return filtered
      .map((part) => {
        const isSelected = part.index === this.selectedPartIndex;
        const colorStyle = part.color ? `background-color: ${part.color};` : 'background-color: #94a3b8;';
        return `
          <div class="part-item ${isSelected ? 'is-selected' : ''}" data-part-index="${part.index}">
            <span class="part-color-dot" style="${colorStyle}"></span>
            <span class="part-name" title="${part.name}">${part.name}</span>
            <span class="part-triangles" title="${part.triangleCount} triangles">${part.triangleCount} Δ</span>
          </div>
        `;
      })
      .join('');
  }

  private bindEvents() {
    const fitBtn = this.container.querySelector('#btn-fit');
    if (fitBtn) {
      fitBtn.addEventListener('click', () => this.events.onFitToModel());
    }

    const resetBtn = this.container.querySelector('#btn-reset');
    if (resetBtn) {
      resetBtn.addEventListener('click', () => this.events.onResetView());
    }

    const wireframeBtn = this.container.querySelector('#btn-wireframe');
    if (wireframeBtn) {
      wireframeBtn.addEventListener('click', () => {
        const newState = this.events.onToggleWireframe();
        this.setWireframeState(newState);
      });
    }

    const segButtons = this.container.querySelectorAll<HTMLButtonElement>('.seg-btn');
    segButtons.forEach((btn) => {
      btn.addEventListener('click', () => {
        const preset = btn.dataset.preset as 'iso' | 'top' | 'front' | 'right';
        if (preset) {
          this.events.onViewPreset(preset);
        }
      });
    });

    const filterInput = this.container.querySelector<HTMLInputElement>('#parts-filter');
    if (filterInput) {
      filterInput.addEventListener('input', (e) => {
        this.filterQuery = (e.target as HTMLInputElement).value;
        const scrollContainer = this.container.querySelector('.parts-scrollable-container');
        if (scrollContainer) {
          scrollContainer.innerHTML = this.renderPartsList();
          this.bindPartItemEvents();
        }
      });
    }

    this.bindPartItemEvents();
  }

  private bindPartItemEvents() {
    const items = this.container.querySelectorAll<HTMLElement>('.part-item');
    items.forEach((item) => {
      item.addEventListener('click', () => {
        const index = parseInt(item.dataset.partIndex || '-1', 10);
        if (index >= 0) {
          this.events.onPartSelect(index);
        }
      });
    });
  }

  private updateSelectionDOM() {
    const items = this.container.querySelectorAll<HTMLElement>('.part-item');
    items.forEach((item) => {
      const index = parseInt(item.dataset.partIndex || '-1', 10);
      const isSelected = index === this.selectedPartIndex;
      item.classList.toggle('is-selected', isSelected);
      if (isSelected) {
        item.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
      }
    });
  }
}
