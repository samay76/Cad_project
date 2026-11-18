import type { PartInfo } from '../types';

export interface PartsListEvents {
  onPartSelect: (index: number) => void;
}

export class PartsListView {
  private container: HTMLElement;
  private events: PartsListEvents;
  private parts: PartInfo[] = [];
  private selectedPartIndex: number | null = null;
  private filterQuery = '';

  constructor(container: HTMLElement, events: PartsListEvents) {
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

  private render() {
    this.container.innerHTML = `
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
