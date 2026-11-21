/**
 * Status bar (bottom): one fixed-height line with the latest message (icon, text, time). The History button
 * opens every earlier message of this session above the bar, as an overlay: the 3D view and the step panel
 * never move. Raw messages stay in Settings > Engineer log.
 */
import type { Friendly } from './friendly';
import { icon } from './icons';

const MAX = 500;
const ICON = { ok: 'checkCircle', info: 'info', warn: 'warn', error: 'error' } as const;

const escapeHtml = (s: string) =>
  s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

type Item = Friendly & { time: string };

export class ActivityCard {
  private items: Item[] = []; // oldest first
  private open = false;
  private row: HTMLElement;
  private history: HTMLElement;
  private list: HTMLElement;
  private toggle: HTMLButtonElement;

  constructor(private container: HTMLElement) {
    container.classList.add('op-status');
    container.innerHTML = `
      <div class="op-status-history" id="op-status-history" role="log" aria-label="Status history" hidden>
        <div class="op-status-history-head">History</div>
        <ul class="op-status-list"></ul>
      </div>
      <div class="op-status-bar">
        <span class="op-status-label">Status</span>
        <div class="op-status-row" id="op-status-row" aria-live="polite"></div>
        <button class="op-status-toggle" id="op-status-toggle" aria-expanded="false" aria-controls="op-status-history">
          <span>History</span>${icon('chevronUp', 18, 'op-chevron')}
        </button>
      </div>`;
    this.row = container.querySelector('#op-status-row')!;
    this.history = container.querySelector('#op-status-history')!;
    this.list = container.querySelector('.op-status-list')!;
    this.toggle = container.querySelector('#op-status-toggle')!;
    this.toggle.addEventListener('click', () => this.setOpen(!this.open));
    document.addEventListener('keydown', (e) => e.key === 'Escape' && this.open && this.setOpen(false));
    // Tap on the row shows the whole message (hover shows it as a tooltip)
    this.row.addEventListener('click', () => this.row.classList.toggle('show-full'));
    this.drawRow();
  }

  public add(event: Friendly) {
    if (!event.text) return;
    const time = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    const last = this.items[this.items.length - 1];
    if (last && last.text === event.text && last.kind === event.kind) {
      last.time = time; // the same line again (polling): only its time changes
      this.drawRow();
      const li = this.list.lastElementChild?.querySelector('time');
      if (li) li.textContent = time;
      return;
    }
    const item = { ...event, time };
    this.items.push(item);
    if (this.items.length > MAX) {
      this.items.shift();
      this.list.firstElementChild?.remove();
    }
    this.drawRow();
    const stick = this.atBottom();
    this.list.appendChild(this.li(item));
    if (stick) this.scrollToEnd();
  }

  /** The messages of this session, oldest first (for tests). */
  public get messages(): readonly Item[] {
    return this.items;
  }

  private li(e: Item) {
    const li = document.createElement('li');
    li.className = `op-event is-${e.kind}`;
    li.innerHTML = `<span class="op-event-icon">${icon(ICON[e.kind], 16)}</span>
      <span class="op-event-text">${escapeHtml(e.text)}</span><time class="op-event-time">${e.time}</time>`;
    return li;
  }

  private drawRow() {
    const e = this.items[this.items.length - 1];
    this.row.className = `op-status-row is-${e ? e.kind : 'empty'}`;
    this.row.title = e ? e.text : '';
    this.row.innerHTML = e
      ? `<span class="op-event-icon">${icon(ICON[e.kind], 18)}</span>
         <span class="op-status-text">${escapeHtml(e.text)}</span><time class="op-event-time">${e.time}</time>`
      : '<span class="op-status-text">Ready. Start by choosing a part.</span>';
  }

  private atBottom() {
    const l = this.history;
    return l.hidden || l.scrollHeight - l.scrollTop - l.clientHeight < 24;
  }

  private scrollToEnd() {
    this.history.scrollTop = this.history.scrollHeight;
  }

  private setOpen(open: boolean) {
    this.open = open;
    this.history.hidden = !open;
    this.toggle.setAttribute('aria-expanded', String(open));
    this.container.classList.toggle('is-open', open);
    if (open) this.scrollToEnd();
  }
}
