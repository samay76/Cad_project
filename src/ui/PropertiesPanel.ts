import type { CADModelMetadata, ClickedPointInfo, SelectionState } from '../types';

export class PropertiesPanel {
  private container: HTMLElement;
  private metadata: CADModelMetadata | null = null;
  private selectionState: SelectionState = {
    selectedPartId: null,
    selectedPartIndex: null,
    selectedPartName: null,
    clickedPoint: null
  };

  constructor(container: HTMLElement) {
    this.container = container;
    this.render();
  }

  public setModelMetadata(metadata: CADModelMetadata | null) {
    this.metadata = metadata;
    this.render();
  }

  public setSelectionState(state: SelectionState) {
    this.selectionState = state;
    this.render();
  }

  private copyToClipboard(text: string, button: HTMLButtonElement) {
    navigator.clipboard.writeText(text).then(() => {
      const originalText = button.innerHTML;
      button.classList.add('copied');
      button.innerHTML = `
        <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
          <polyline points="20 6 9 17 4 12"/>
        </svg>
        <span>Copied!</span>
      `;
      setTimeout(() => {
        button.classList.remove('copied');
        button.innerHTML = originalText;
      }, 1800);
    }).catch(() => {
      // Fallback
      prompt('Copy coordinates:', text);
    });
  }

  private render() {
    this.container.innerHTML = `
      <div class="sidebar-block properties-block">
        <div class="block-header">
          <span class="block-title">CAD PROPERTIES</span>
        </div>

        <div class="properties-scrollable">
          <!-- 1. MODEL OVERVIEW -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"/>
              </svg>
              <span>Model Overview</span>
            </div>

            ${this.renderModelInfo()}
          </div>

          <!-- 2. BOUNDING BOX SIZE -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="3" y="3" width="18" height="18" rx="2" ry="2"/>
                <line x1="3" y1="9" x2="21" y2="9"/>
                <line x1="9" y1="21" x2="9" y2="9"/>
              </svg>
              <span>Bounding Box Size (mm)</span>
            </div>

            ${this.renderBoundingBoxInfo()}
          </div>

          <!-- 3. SELECTED PART -->
          <div class="property-section">
            <div class="section-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <polygon points="12 2 2 7 12 12 22 7 12 2"/>
              </svg>
              <span>Selected Part</span>
            </div>

            ${this.renderSelectedPartInfo()}
          </div>

          <!-- 4. CLICKED SURFACE PICK-POINT -->
          <div class="property-section highlight-section">
            <div class="section-title pick-point-title">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <circle cx="12" cy="12" r="10"/>
                <circle cx="12" cy="12" r="3"/>
                <line x1="12" y1="2" x2="12" y2="5"/>
                <line x1="12" y1="19" x2="12" y2="22"/>
                <line x1="2" y1="12" x2="5" y2="12"/>
                <line x1="19" y1="12" x2="22" y2="12"/>
              </svg>
              <span>Surface Pick Point</span>
            </div>

            ${this.renderClickedPointInfo()}
          </div>
        </div>
      </div>
    `;

    this.bindCopyButtons();
  }

  private renderModelInfo(): string {
    if (!this.metadata) {
      return `
        <div class="empty-property-state">
          <span>No CAD file loaded</span>
        </div>
      `;
    }

    const typeBadgeClass = `badge-${this.metadata.fileType}`;

    return `
      <div class="kv-grid">
        <div class="kv-row">
          <span class="kv-label">File Name</span>
          <span class="kv-value mono file-name-value" title="${this.metadata.fileName}">${this.metadata.fileName}</span>
        </div>
        <div class="kv-row">
          <span class="kv-label">Format</span>
          <span class="kv-value"><span class="badge ${typeBadgeClass}">${this.metadata.fileType.toUpperCase()}</span></span>
        </div>
        <div class="kv-row">
          <span class="kv-label">Parts / Solids</span>
          <span class="kv-value font-semibold">${this.metadata.partsCount}</span>
        </div>
        <div class="kv-row">
          <span class="kv-label">Total Triangles</span>
          <span class="kv-value mono">${this.metadata.triangleCount.toLocaleString()}</span>
        </div>
      </div>
    `;
  }

  private renderBoundingBoxInfo(): string {
    if (!this.metadata) {
      return `
        <div class="empty-property-state">
          <span>Awaiting model...</span>
        </div>
      `;
    }

    const size = this.metadata.boundingBox.size;

    return `
      <div class="bbox-coords-box">
        <div class="coord-item">
          <span class="coord-axis axis-x">X</span>
          <span class="coord-val mono">${size.x.toFixed(3)} mm</span>
        </div>
        <div class="coord-item">
          <span class="coord-axis axis-y">Y</span>
          <span class="coord-val mono">${size.y.toFixed(3)} mm</span>
        </div>
        <div class="coord-item">
          <span class="coord-axis axis-z">Z</span>
          <span class="coord-val mono">${size.z.toFixed(3)} mm</span>
        </div>
      </div>
    `;
  }

  private renderSelectedPartInfo(): string {
    if (this.selectionState.selectedPartIndex === null) {
      return `
        <div class="empty-property-state">
          <span>No part selected</span>
          <span class="hint-text">Click a part in 3D view or parts list</span>
        </div>
      `;
    }

    const idx = this.selectionState.selectedPartIndex;
    const name = this.selectionState.selectedPartName || `Part #${idx + 1}`;

    return `
      <div class="kv-grid">
        <div class="kv-row">
          <span class="kv-label">Part Name</span>
          <span class="kv-value font-semibold" title="${name}">${name}</span>
        </div>
        <div class="kv-row">
          <span class="kv-label">Index / Solid #</span>
          <span class="kv-value mono">#${idx}</span>
        </div>
      </div>
    `;
  }

  private renderClickedPointInfo(): string {
    const pt = this.selectionState.clickedPoint;

    if (!pt) {
      return `
        <div class="empty-property-state pick-point-empty">
          <span>No surface point selected</span>
          <span class="hint-text">Click anywhere on the CAD model surface to record pick coordinates</span>
        </div>
      `;
    }

    const pointCoordsStr = `X: ${pt.point.x.toFixed(3)}, Y: ${pt.point.y.toFixed(3)}, Z: ${pt.point.z.toFixed(3)}`;
    const normalStr = `[${pt.normal.x.toFixed(3)}, ${pt.normal.y.toFixed(3)}, ${pt.normal.z.toFixed(3)}]`;

    return `
      <div class="pick-point-container">
        <!-- Point Coordinates -->
        <div class="pick-coord-card">
          <div class="card-top">
            <span class="sub-label">Model Frame Coordinates (mm)</span>
            <button class="btn-copy" data-copy-text="${pointCoordsStr}" title="Copy XYZ Coordinates">
              <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="9" y="9" width="13" height="13" rx="2" ry="2"/>
                <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"/>
              </svg>
              <span>Copy</span>
            </button>
          </div>
          <div class="coord-readout-grid">
            <div class="coord-readout-item">
              <span class="axis-tag axis-x">X</span>
              <span class="coord-num mono">${pt.point.x.toFixed(3)}</span>
              <span class="unit-tag">mm</span>
            </div>
            <div class="coord-readout-item">
              <span class="axis-tag axis-y">Y</span>
              <span class="coord-num mono">${pt.point.y.toFixed(3)}</span>
              <span class="unit-tag">mm</span>
            </div>
            <div class="coord-readout-item">
              <span class="axis-tag axis-z">Z</span>
              <span class="coord-num mono">${pt.point.z.toFixed(3)}</span>
              <span class="unit-tag">mm</span>
            </div>
          </div>
        </div>

        <!-- Normal Vector -->
        <div class="pick-coord-card">
          <div class="card-top">
            <span class="sub-label">Face Normal (Unit Vector)</span>
            <button class="btn-copy" data-copy-text="${normalStr}" title="Copy Normal Vector">
              <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="9" y="9" width="13" height="13" rx="2" ry="2"/>
                <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"/>
              </svg>
              <span>Copy</span>
            </button>
          </div>
          <div class="coord-readout-grid normal-grid">
            <div class="coord-readout-item">
              <span class="axis-tag">Nx</span>
              <span class="coord-num mono">${pt.normal.x.toFixed(3)}</span>
            </div>
            <div class="coord-readout-item">
              <span class="axis-tag">Ny</span>
              <span class="coord-num mono">${pt.normal.y.toFixed(3)}</span>
            </div>
            <div class="coord-readout-item">
              <span class="axis-tag">Nz</span>
              <span class="coord-num mono">${pt.normal.z.toFixed(3)}</span>
            </div>
          </div>
        </div>

        <!-- On Part info -->
        <div class="pick-target-part">
          <span class="target-label">On:</span>
          <span class="target-name font-semibold">${pt.partName} (#${pt.partIndex})</span>
        </div>
      </div>
    `;
  }

  private bindCopyButtons() {
    const copyBtns = this.container.querySelectorAll<HTMLButtonElement>('.btn-copy');
    copyBtns.forEach((btn) => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const text = btn.dataset.copyText || '';
        if (text) {
          this.copyToClipboard(text, btn);
        }
      });
    });
  }
}
