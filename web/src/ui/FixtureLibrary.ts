/**
 * Fixture library screens: fixture cards in the Load part step, and the fixture window (its STEP parts:
 * add, use, rename, remove). Data lives on the cell server (library/api.ts -> server/library.py).
 */
import { library, STEP_FILE, type Fixture, type LibraryPart } from '../library/api';
import { icon } from './icons';

export interface FixtureLibraryEvents {
  /** "Use this part": load it into the 3D view (and restore its grip points). */
  onUsePart: (fixture: Fixture, part: LibraryPart) => Promise<void>;
  /** A library change worth a line in the Status bar. */
  onNotice: (text: string, kind: 'ok' | 'info' | 'warn' | 'error') => void;
  /** The fixture or part in use was renamed or removed. */
  onSelectionChanged: () => void;
}

const escapeHtml = (s: string) =>
  s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

const date = (iso: string) => {
  const d = new Date(iso);
  return isNaN(d.getTime()) ? '' : d.toLocaleDateString([], { day: 'numeric', month: 'short', year: 'numeric' });
};

const size = (bytes: number) => (bytes > 1e6 ? `${(bytes / 1e6).toFixed(1)} MB` : `${Math.max(1, Math.round(bytes / 1e3))} kB`);

/** A small in-app dialog asking for a name (window.prompt does not exist in the desktop app). */
export function askName(title: string, label: string, initial = '', validate?: (v: string) => string | null): Promise<string | null> {
  return new Promise((resolve) => {
    const wrap = document.createElement('div');
    wrap.className = 'op-modal-wrap op-ask';
    wrap.innerHTML = `
      <form class="op-modal op-ask-box" role="dialog" aria-modal="true" aria-label="${escapeHtml(title)}">
        <h3 class="op-modal-title">${escapeHtml(title)}</h3>
        <label class="op-form-label">${escapeHtml(label)}<input type="text" maxlength="80" value="${escapeHtml(initial)}" required></label>
        <div class="op-ask-error" role="alert"></div>
        <div class="op-ask-actions">
          <button type="button" class="btn btn-secondary op-btn" data-cancel>Cancel</button>
          <button type="submit" class="btn btn-primary op-btn">Save</button>
        </div>
      </form>`;
    document.body.appendChild(wrap);
    const input = wrap.querySelector('input')!;
    const err = wrap.querySelector<HTMLElement>('.op-ask-error')!;
    const done = (v: string | null) => {
      wrap.remove();
      resolve(v);
    };
    input.focus();
    input.select();
    wrap.querySelector('[data-cancel]')!.addEventListener('click', () => done(null));
    wrap.addEventListener('keydown', (e) => e.key === 'Escape' && done(null));
    wrap.addEventListener('mousedown', (e) => e.target === wrap && done(null));
    wrap.querySelector('form')!.addEventListener('submit', (e) => {
      e.preventDefault();
      const v = input.value.trim().replace(/\s+/g, ' ');
      const problem = !v ? 'A name is required.' : validate?.(v) ?? null;
      if (problem) {
        err.textContent = problem;
        input.focus();
        return;
      }
      done(v);
    });
  });
}

/** New fixture: its CAD file (required, STEP) and a name (from the file name unless changed). */
export function askFixture(taken: (v: string) => string | null): Promise<{ name: string; file: File } | null> {
  return new Promise((resolve) => {
    const wrap = document.createElement('div');
    wrap.className = 'op-modal-wrap op-ask';
    wrap.innerHTML = `
      <form class="op-modal op-ask-box" role="dialog" aria-modal="true" aria-label="Add fixture">
        <h3 class="op-modal-title">Add fixture</h3>
        <p class="op-small">First the fixture's own CAD file. Then you add the parts that sit in it.</p>
        <div class="op-form-label">Fixture CAD file (STEP)
          <button type="button" class="op-file-pick" data-pick>${icon('upload', 18)}<span>Choose the fixture STEP file</span></button>
          <input type="file" class="hidden-file-input" accept=".step,.stp,.STEP,.STP">
        </div>
        <label class="op-form-label">Fixture name<input type="text" name="name" maxlength="80" placeholder="e.g. F1"></label>
        <div class="op-ask-error" role="alert"></div>
        <div class="op-ask-actions">
          <button type="button" class="btn btn-secondary op-btn" data-cancel>Cancel</button>
          <button type="submit" class="btn btn-primary op-btn">Add fixture</button>
        </div>
      </form>`;
    document.body.appendChild(wrap);
    const fileInput = wrap.querySelector<HTMLInputElement>('input[type=file]')!;
    const nameInput = wrap.querySelector<HTMLInputElement>('input[name=name]')!;
    const pick = wrap.querySelector<HTMLButtonElement>('[data-pick]')!;
    const err = wrap.querySelector<HTMLElement>('.op-ask-error')!;
    let file: File | null = null;
    let nameTyped = false;
    const done = (v: { name: string; file: File } | null) => {
      wrap.remove();
      resolve(v);
    };
    const setFile = (f: File | undefined) => {
      if (!f) return;
      if (!STEP_FILE.test(f.name)) {
        err.textContent = 'Only STEP files can be added.';
        return;
      }
      file = f;
      err.textContent = '';
      pick.classList.add('has-file');
      pick.innerHTML = `${icon('file', 18)}<span>${escapeHtml(f.name)}</span><span class="op-small">${size(f.size)} · change</span>`;
      if (!nameTyped) nameInput.value = f.name.replace(/\.(step|stp)$/i, '');
    };
    pick.addEventListener('click', () => {
      fileInput.value = '';
      fileInput.click();
    });
    fileInput.addEventListener('change', () => setFile(fileInput.files?.[0]));
    wrap.addEventListener('dragover', (e) => e.preventDefault());
    wrap.addEventListener('drop', (e) => {
      e.preventDefault();
      setFile(e.dataTransfer?.files[0]);
    });
    nameInput.addEventListener('input', () => (nameTyped = true));
    wrap.querySelector('[data-cancel]')!.addEventListener('click', () => done(null));
    wrap.addEventListener('keydown', (e) => e.key === 'Escape' && done(null));
    wrap.addEventListener('mousedown', (e) => e.target === wrap && done(null));
    wrap.querySelector('form')!.addEventListener('submit', (e) => {
      e.preventDefault();
      const name = nameInput.value.trim().replace(/\s+/g, ' ');
      const problem = !file ? 'Choose the fixture CAD file first.' : !name ? 'A name is required.' : taken(name);
      if (problem) {
        err.textContent = problem;
        return;
      }
      done({ name, file: file! });
    });
    pick.focus();
  });
}

export class FixtureLibrary {
  private fixtures: Fixture[] = [];
  private loaded = false;
  private error: string | null = null;
  private mount: HTMLElement | null = null;
  private openId: string | null = null; // fixture shown in the window
  private window: HTMLDivElement | null = null;
  private working: string | null = null; // part id being loaded / "upload"
  /** The part in use (from "Use this part"). */
  public current: { fixtureId: string; partId: string } | null = null;

  constructor(private events: FixtureLibraryEvents) {
    this.reload();
  }

  public async reload() {
    try {
      this.fixtures = await library.list();
      this.error = null;
    } catch (e: any) {
      this.error = e?.message || String(e);
    }
    this.loaded = true;
    this.redraw();
  }

  /** The fixture and part in use, for the header ("Fixture F1 · Part2"). */
  public currentNames(): { fixture: string; part: string } | null {
    if (!this.current) return null;
    const fx = this.fixtures.find((f) => f.id === this.current!.fixtureId);
    const p = fx?.parts.find((x) => x.id === this.current!.partId);
    return fx && p ? { fixture: fx.name, part: p.name } : null;
  }

  public currentPart(): { fixture: Fixture; part: LibraryPart } | null {
    if (!this.current) return null;
    const fixture = this.fixtures.find((f) => f.id === this.current!.fixtureId);
    const part = fixture?.parts.find((x) => x.id === this.current!.partId);
    return fixture && part ? { fixture, part } : null;
  }

  /** Keeps the local copy in step after the grip points were saved on the server. */
  public setGrip(partId: string, grip: LibraryPart['grip']) {
    for (const fx of this.fixtures) for (const p of fx.parts) if (p.id === partId) p.grip = grip;
  }

  // ---------------- cards in the Load part step ----------------
  public render(el: HTMLElement) {
    this.mount = el;
    this.drawCards();
  }

  private redraw() {
    if (this.mount?.isConnected) this.drawCards();
    if (this.window) this.drawWindow();
  }

  private drawCards() {
    const el = this.mount!;
    const using = this.current;
    el.innerHTML = `
      <div class="op-fixtures-head">
        <span class="op-sub">Fixtures</span>
        <button class="btn btn-secondary op-btn-sm" id="op-add-fixture">${icon('plus', 16)}<span>Add fixture</span></button>
      </div>
      ${
        !this.loaded
          ? '<div class="op-small">Loading fixtures…</div>'
          : this.error
            ? `<div class="op-result is-error"><span class="op-result-icon">${icon('error', 20)}</span><div><div class="op-result-title">Fixtures could not be loaded</div><div class="op-result-line">${escapeHtml(this.error)}</div></div></div>`
            : this.fixtures.length
              ? `<div class="op-fixture-grid">${this.fixtures
                  .map(
                    (f) => `<button class="op-fixture-card ${using?.fixtureId === f.id ? 'is-current' : ''}" data-fixture="${f.id}">
                      <span class="op-fixture-icon">${icon('box', 22)}</span>
                      <span class="op-fixture-name">${escapeHtml(f.name)}</span>
                      <span class="op-fixture-count">${f.file ? `${f.parts.length} part${f.parts.length === 1 ? '' : 's'}` : `<span class="op-warn-text">${icon('warn', 14)} No fixture CAD</span>`}</span>
                    </button>`
                  )
                  .join('')}</div>`
              : `<div class="op-empty">${icon('box', 28)}<span>No fixtures yet. Add a fixture with its CAD file, then the parts that sit in it.</span></div>`
      }`;
    el.querySelector('#op-add-fixture')!.addEventListener('click', () => this.addFixture());
    el.querySelectorAll<HTMLButtonElement>('[data-fixture]').forEach((b) =>
      b.addEventListener('click', () => this.openFixture(b.dataset.fixture!))
    );
  }

  private uniqueFixture = (skip?: string) => (v: string) =>
    this.fixtures.some((f) => f.id !== skip && f.name.toLowerCase() === v.toLowerCase()) ? `A fixture named "${v}" already exists.` : null;

  private async addFixture() {
    const answer = await askFixture(this.uniqueFixture());
    if (!answer) return;
    try {
      const fx = await library.addFixture(answer.name, answer.file);
      this.fixtures.push(fx);
      this.events.onNotice(`Fixture ${fx.name} added (${answer.file.name}). Now add the parts that sit in it.`, 'ok');
      this.redraw();
      this.openFixture(fx.id);
    } catch (e: any) {
      this.events.onNotice(e?.message || String(e), 'error');
    }
  }

  // ---------------- fixture window ----------------
  private openFixture(id: string) {
    this.openId = id;
    if (!this.window) {
      this.window = document.createElement('div');
      this.window.className = 'op-modal-wrap';
      this.window.addEventListener('mousedown', (e) => e.target === this.window && this.closeWindow());
      this.window.addEventListener('keydown', (e) => e.key === 'Escape' && !document.querySelector('.op-ask') && this.closeWindow());
      document.body.appendChild(this.window);
    }
    this.drawWindow();
    this.window.querySelector<HTMLElement>('.op-modal')?.focus();
  }

  private closeWindow() {
    this.window?.remove();
    this.window = null;
    this.openId = null;
  }

  private drawWindow() {
    const fx = this.fixtures.find((f) => f.id === this.openId);
    if (!fx || !this.window) return this.closeWindow();
    const busy = this.working !== null;
    this.window.innerHTML = `
      <div class="op-modal op-fixture-window" role="dialog" aria-modal="true" aria-label="Fixture ${escapeHtml(fx.name)}" tabindex="-1">
        <div class="op-modal-head">
          <div class="op-fixture-title">
            <span class="op-fixture-icon">${icon('box', 22)}</span>
            <h3 class="op-modal-title">Fixture ${escapeHtml(fx.name)}</h3>
            <button class="op-icon-btn op-icon-sm" data-act="rename-fixture" title="Rename fixture" aria-label="Rename fixture">${icon('pencil', 16)}</button>
          </div>
          <button class="op-icon-btn" data-act="close" aria-label="Close">${icon('close', 22)}</button>
        </div>
        <div class="op-modal-body">
          <div class="op-sub">Fixture CAD</div>
          ${
            fx.file
              ? `<div class="op-part-row op-fixture-file">
                  <span class="op-part-thumb op-thumb-fixture">${icon('box', 22)}</span>
                  <span class="op-part-info">
                    <span class="op-part-name">${escapeHtml(fx.file.fileName)}</span>
                    <span class="op-part-meta">${size(fx.file.size)} · added ${date(fx.file.uploadedAt)} · the parts below sit in it</span>
                  </span>
                  <button class="btn btn-secondary op-btn-sm" data-act="fixture-file" ${busy ? 'disabled' : ''}>${this.working === 'fixture' ? 'Uploading…' : 'Replace'}</button>
                </div>`
              : `<div class="op-result is-warn"><span class="op-result-icon">${icon('warn', 20)}</span>
                  <div style="flex:1"><div class="op-result-title">Add the fixture CAD file</div>
                  <div class="op-result-line">The fixture's own STEP file. The parts are placed in it in the 3D view and in the simulation.</div></div>
                  <button class="btn btn-primary op-btn-sm" data-act="fixture-file" ${busy ? 'disabled' : ''}>${this.working === 'fixture' ? 'Uploading…' : 'Add fixture CAD'}</button>
                </div>`
          }
          <div class="op-row">
            <span class="op-sub">Parts in this fixture (${fx.parts.length})</span>
            <button class="btn btn-primary op-btn-sm" data-act="add-part" ${busy ? 'disabled' : ''}>${icon('upload', 16)}<span>${this.working === 'upload' ? 'Adding…' : 'Add CAD model'}</span></button>
          </div>
          ${
            fx.parts.length
              ? `<ul class="op-part-list">${fx.parts
                  .map((p) => {
                    const inUse = this.current?.partId === p.id;
                    return `<li class="op-part-row ${inUse ? 'is-current' : ''}">
                      <span class="op-part-thumb">${icon('file', 22)}</span>
                      <span class="op-part-info">
                        <span class="op-part-name">${escapeHtml(p.name)}${inUse ? ' <span class="op-tag">In use</span>' : ''}${p.grip ? ` <span class="op-tag is-ok" title="Grip points saved">${icon('check', 12)} Grip saved</span>` : ''}</span>
                        <span class="op-part-meta">${escapeHtml(p.fileName)} · ${size(p.size)} · added ${date(p.uploadedAt)}</span>
                      </span>
                      <span class="op-part-actions">
                        <button class="btn btn-primary op-btn-sm" data-act="use" data-part="${p.id}" ${busy || !fx.file ? 'disabled' : ''} title="${fx.file ? '' : 'Add the fixture CAD file first.'}">${this.working === p.id ? 'Loading…' : 'Use this part'}</button>
                        <button class="op-icon-btn op-icon-sm" data-act="rename" data-part="${p.id}" title="Rename" aria-label="Rename ${escapeHtml(p.name)}" ${busy ? 'disabled' : ''}>${icon('pencil', 16)}</button>
                        <button class="op-icon-btn op-icon-sm op-danger" data-act="remove" data-part="${p.id}" title="Remove" aria-label="Remove ${escapeHtml(p.name)}" ${busy ? 'disabled' : ''}>${icon('trash', 16)}</button>
                      </span>
                    </li>`;
                  })
                  .join('')}</ul>`
              : `<div class="op-empty op-drop-parts" data-act="add-part">${icon('upload', 28)}<span>Add the first CAD model for this fixture</span><span class="op-small">The parts that sit in ${escapeHtml(fx.name)}. STEP files (.step, .stp); you can also drop them here.</span></div>`
          }
        </div>
        <div class="op-modal-foot">
          <button class="btn btn-text op-btn op-danger-text" data-act="delete-fixture" ${busy ? 'disabled' : ''}>${icon('trash', 16)}<span>Delete fixture</span></button>
          <input type="file" class="hidden-file-input" data-parts accept=".step,.stp,.STEP,.STP" multiple>
          <input type="file" class="hidden-file-input" data-fixture-file accept=".step,.stp,.STEP,.STP">
        </div>
      </div>`;
    const w = this.window;
    const input = w.querySelector<HTMLInputElement>('input[data-parts]')!;
    input.addEventListener('change', () => input.files && this.addParts(fx, Array.from(input.files)));
    const fixtureInput = w.querySelector<HTMLInputElement>('input[data-fixture-file]')!;
    fixtureInput.addEventListener('change', () => fixtureInput.files?.[0] && this.setFixtureFile(fx, fixtureInput.files[0]));
    const body = w.querySelector<HTMLElement>('.op-modal-body')!;
    body.addEventListener('dragover', (e) => {
      e.preventDefault();
      body.classList.add('is-over');
    });
    body.addEventListener('dragleave', () => body.classList.remove('is-over'));
    body.addEventListener('drop', (e) => {
      e.preventDefault();
      body.classList.remove('is-over');
      if (e.dataTransfer?.files.length) this.addParts(fx, Array.from(e.dataTransfer.files));
    });
    w.querySelectorAll<HTMLElement>('[data-act]').forEach((b) =>
      b.addEventListener('click', () => {
        const part = fx.parts.find((p) => p.id === b.dataset.part);
        switch (b.dataset.act) {
          case 'close':
            return this.closeWindow();
          case 'add-part':
            if (this.working) return;
            input.value = '';
            return input.click();
          case 'fixture-file':
            if (this.working) return;
            fixtureInput.value = '';
            return fixtureInput.click();
          case 'rename-fixture':
            return this.renameFixture(fx);
          case 'delete-fixture':
            return this.deleteFixture(fx);
          case 'use':
            return part && this.usePart(fx, part);
          case 'rename':
            return part && this.renamePart(fx, part);
          case 'remove':
            return part && this.removePart(fx, part);
        }
      })
    );
  }

  private async addParts(fx: Fixture, files: File[]) {
    if (this.working) return;
    const bad = files.filter((f) => !STEP_FILE.test(f.name));
    if (bad.length) this.events.onNotice(`Only STEP files can be added (${bad.map((f) => f.name).join(', ')}).`, 'error');
    this.working = 'upload';
    this.drawWindow();
    for (const file of files.filter((f) => STEP_FILE.test(f.name))) {
      try {
        const part = await library.addPart(fx.id, file);
        fx.parts.push(part);
        this.events.onNotice(`${part.name} added to fixture ${fx.name}`, 'ok');
      } catch (e: any) {
        this.events.onNotice(e?.message || String(e), 'error');
      }
    }
    this.working = null;
    this.redraw();
  }

  private async setFixtureFile(fx: Fixture, file: File) {
    if (fx.file && !window.confirm(`Replace the CAD file of fixture ${fx.name} (${fx.file.fileName}) with ${file.name}?`)) return;
    this.working = 'fixture';
    this.drawWindow();
    try {
      const updated = await library.setFixtureFile(fx.id, file);
      fx.file = updated.file;
      this.events.onNotice(`Fixture ${fx.name} now uses ${file.name}`, 'ok');
      if (this.current?.fixtureId === fx.id) this.events.onSelectionChanged();
    } catch (e: any) {
      this.events.onNotice(e?.message || String(e), 'error');
    }
    this.working = null;
    this.redraw();
  }

  private async usePart(fx: Fixture, part: LibraryPart) {
    this.working = part.id;
    this.drawWindow();
    try {
      await this.events.onUsePart(fx, part);
      this.current = { fixtureId: fx.id, partId: part.id };
      this.working = null;
      this.closeWindow();
      this.events.onSelectionChanged();
    } catch (e: any) {
      this.working = null;
      this.events.onNotice(e?.message || String(e), 'error');
    }
    this.redraw();
  }

  private async renameFixture(fx: Fixture) {
    const name = await askName('Rename fixture', 'Fixture name', fx.name, this.uniqueFixture(fx.id));
    if (!name || name === fx.name) return;
    try {
      fx.name = (await library.renameFixture(fx.id, name)).name;
      this.redraw();
      this.events.onSelectionChanged();
    } catch (e: any) {
      this.events.onNotice(e?.message || String(e), 'error');
    }
  }

  private async deleteFixture(fx: Fixture) {
    const n = fx.parts.length;
    if (!window.confirm(`Delete fixture ${fx.name}${n ? ` and its ${n} CAD model${n === 1 ? '' : 's'}` : ''}? This cannot be undone.`)) return;
    try {
      await library.deleteFixture(fx.id);
      this.fixtures = this.fixtures.filter((f) => f.id !== fx.id);
      if (this.current?.fixtureId === fx.id) this.current = null;
      this.closeWindow();
      this.redraw();
      this.events.onNotice(`Fixture ${fx.name} deleted`, 'info');
      this.events.onSelectionChanged();
    } catch (e: any) {
      this.events.onNotice(e?.message || String(e), 'error');
    }
  }

  private async renamePart(fx: Fixture, part: LibraryPart) {
    const unique = (v: string) =>
      fx.parts.some((p) => p.id !== part.id && p.name.toLowerCase() === v.toLowerCase()) ? `This fixture already has a part named "${v}".` : null;
    const name = await askName('Rename part', 'Part name', part.name, unique);
    if (!name || name === part.name) return;
    try {
      part.name = (await library.renamePart(fx.id, part.id, name)).name;
      this.redraw();
      this.events.onSelectionChanged();
    } catch (e: any) {
      this.events.onNotice(e?.message || String(e), 'error');
    }
  }

  private async removePart(fx: Fixture, part: LibraryPart) {
    if (!window.confirm(`Remove ${part.name} from fixture ${fx.name}? This cannot be undone.`)) return;
    try {
      await library.deletePart(fx.id, part.id);
      fx.parts = fx.parts.filter((p) => p.id !== part.id);
      if (this.current?.partId === part.id) this.current = null;
      this.redraw();
      this.events.onNotice(`${part.name} removed from fixture ${fx.name}`, 'info');
      this.events.onSelectionChanged();
    } catch (e: any) {
      this.events.onNotice(e?.message || String(e), 'error');
    }
  }
}
