/**
 * Log window (bottom of the right sidebar): every message of planning, Sim and Real Sim, timestamped, newest at the
 * bottom. Repeats of the same message are not added again.
 */
export type LogKind = 'info' | 'ok' | 'error';

const escapeHtml = (s: string) =>
  s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

export class LogPanel {
  private list: HTMLElement;
  private last = '';

  constructor(container: HTMLElement) {
    container.innerHTML = `
      <div class="sidebar-block log-block">
        <div class="block-header">
          <span class="block-title">LOG</span>
          <button class="btn btn-secondary btn-sm log-clear" title="Clear the log">Clear</button>
        </div>
        <div id="cell-log" class="log-list"></div>
      </div>`;
    this.list = container.querySelector('#cell-log')!;
    container.querySelector('.log-clear')!.addEventListener('click', () => {
      this.list.innerHTML = '';
      this.last = '';
    });
  }

  public add(text: string, kind: LogKind = 'info') {
    const key = `${kind}|${text}`;
    if (!text || key === this.last) return;
    this.last = key;
    const time = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    const row = document.createElement('div');
    row.className = `log-entry is-${kind}`;
    row.innerHTML = `<span class="log-time">${time}</span><span class="log-text">${escapeHtml(text)}</span>`;
    this.list.appendChild(row);
    while (this.list.children.length > 300) this.list.firstElementChild!.remove();
    this.list.scrollTop = this.list.scrollHeight;
  }
}
