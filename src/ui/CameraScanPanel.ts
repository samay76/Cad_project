/**
 * Viewport panel for building the part from the depth camera instead of loading a CAD file:
 *
 *   [ live depth camera ]  [ CAD model built from the camera  |  live object ]
 *   Build CAD model  ->  check the model on the right  ->  Load model for gripper  ->  mark the finger points.
 *
 * Talks to camera_scan_server.py through the dev-server bridge (vite-plugins/cameraScanBridge.ts, /api/scan/*).
 */
import { ScanModelPreview } from './ScanModelPreview';

interface ScanStatus {
  camera: 'starting' | 'streaming' | 'error';
  message: string;
  fps: number;
  points: number;
  table: boolean;
  distanceM: number | null;
  sizeMm: number[] | null;
  maxDepthM: number;
  building: { frames: number; of: number } | null;
  model: { id: string; file: string; sizeMm: number[]; triangles: number } | null;
  error: string | null;
}

interface BuiltModel {
  id: string;
  file: string;
  sizeMm: number[];
  buffer: ArrayBuffer;
}

export interface CameraScanEvents {
  /** The user chose to use the built model: load it as the part for the gripper. */
  onLoadModel: (buffer: ArrayBuffer, fileName: string) => Promise<void>;
}

const POLL_MS = 400;
const STREAM_RETRY_MS = 1000;
type RightView = 'model' | 'live';

export class CameraScanPanel {
  private el: HTMLElement;
  private depthImg: HTMLImageElement;
  private liveImg: HTMLImageElement;
  private modelView: HTMLElement;
  private modelEmpty: HTMLElement;
  private statusEl: HTMLElement;
  private infoEl: HTMLElement;
  private buildBtn: HTMLButtonElement;
  private useBtn: HTMLButtonElement;
  private preview: ScanModelPreview;
  private open = false;
  private rightView: RightView = 'live';
  private pollTimer: number | null = null;
  private lastStatus: ScanStatus | null = null;
  private knownModelId: string | null | undefined = undefined; // undefined = not polled yet
  private built: BuiltModel | null = null;
  private loadedModelId: string | null = null;
  private busy = false;

  constructor(
    parent: HTMLElement,
    private events: CameraScanEvents
  ) {
    this.el = document.createElement('div');
    this.el.className = 'camera-scan-panel';
    this.el.innerHTML = `
      <div class="camera-feed-header">
        <span class="camera-feed-live-dot"></span>
        <span class="camera-feed-title">Scan part from depth camera</span>
        <button class="camera-feed-close" title="Close and release the camera">&times;</button>
      </div>
      <div class="camera-scan-feeds">
        <figure class="camera-scan-feed">
          <div class="camera-scan-view">
            <img data-feed="depth" alt="Depth camera feed" />
            <div class="camera-feed-placeholder">Waiting for the camera...</div>
          </div>
          <figcaption>Depth camera <span class="camera-scan-hint">red = near · black = no depth</span></figcaption>
        </figure>
        <figure class="camera-scan-feed">
          <div class="camera-scan-view" data-role="right-view">
            <img data-feed="live" alt="Live object point cloud" />
            <div class="camera-feed-placeholder" data-role="live-placeholder">Waiting for the camera...</div>
            <div class="camera-scan-model" data-role="model-view"></div>
            <div class="camera-feed-placeholder" data-role="model-empty">No CAD model yet.<br/>Press <b>Build CAD model</b>.</div>
          </div>
          <figcaption>
            <span class="camera-scan-tabs" role="group" aria-label="Right view">
              <button class="camera-scan-tab" data-right="model" title="The CAD model built from the camera (drag to rotate)">CAD model</button>
              <button class="camera-scan-tab" data-right="live" title="Live: the object's point cloud on the camera image (blue tint = table)">Live object</button>
            </span>
            <span class="camera-scan-hint" data-role="right-hint"></span>
          </figcaption>
        </figure>
      </div>
      <div class="camera-scan-info"></div>
      <div class="camera-scan-actions">
        <button class="btn btn-secondary btn-sm" data-act="build" title="Fuse 30 frames into a solid model. Keep the camera and the part still.">Build CAD model</button>
        <button class="btn btn-primary btn-sm" data-act="use" title="Put this model in the 3D view so you can mark where the gripper holds it">Load model for gripper</button>
        <button class="btn btn-secondary btn-sm" data-act="refit" title="Find the table plane again (after moving the camera)">Re-fit table</button>
        <span class="camera-scan-depth" title="Maximum depth: set it just behind the part to cut away the room">
          <button class="btn btn-secondary btn-sm" data-act="depth-" aria-label="Decrease max depth">−</button>
          <span data-role="max-depth">0.80 m</span>
          <button class="btn btn-secondary btn-sm" data-act="depth+" aria-label="Increase max depth">+</button>
        </span>
      </div>
      <div class="camera-scan-status" aria-live="polite"></div>
    `;
    parent.appendChild(this.el);

    this.depthImg = this.el.querySelector('img[data-feed="depth"]')!;
    this.liveImg = this.el.querySelector('img[data-feed="live"]')!;
    this.modelView = this.el.querySelector('[data-role="model-view"]')!;
    this.modelEmpty = this.el.querySelector('[data-role="model-empty"]')!;
    this.statusEl = this.el.querySelector('.camera-scan-status')!;
    this.infoEl = this.el.querySelector('.camera-scan-info')!;
    this.buildBtn = this.el.querySelector('[data-act="build"]')!;
    this.useBtn = this.el.querySelector('[data-act="use"]')!;
    this.preview = new ScanModelPreview(this.modelView);

    // Feed frames must not be dragged into the 3D view (it would try to load the JPEG as a CAD file).
    this.el.addEventListener('dragstart', (e) => e.preventDefault());

    for (const img of [this.depthImg, this.liveImg]) {
      img.draggable = false;
      const placeholder = img.nextElementSibling as HTMLElement;
      img.addEventListener('load', () => {
        placeholder.style.display = 'none';
        img.style.display = 'block';
      });
      // The server may still be starting (or the camera reconnecting): retry the stream.
      img.addEventListener('error', () => {
        img.style.display = 'none';
        placeholder.style.display = 'flex';
        if (this.open && img.getAttribute('src')) window.setTimeout(() => this.connectStream(img), STREAM_RETRY_MS);
      });
    }

    this.el.querySelector('.camera-feed-close')!.addEventListener('click', () => this.close());
    this.el.querySelectorAll<HTMLButtonElement>('.camera-scan-tab').forEach((btn) =>
      btn.addEventListener('click', () => this.setRightView(btn.dataset.right as RightView))
    );
    this.buildBtn.addEventListener('click', () => this.build());
    this.useBtn.addEventListener('click', () => this.useModel());
    this.el.querySelector('[data-act="refit"]')!.addEventListener('click', () => this.post('refit'));
    this.el.querySelector('[data-act="depth-"]')!.addEventListener('click', () => this.post('depth', { delta: -0.05 }));
    this.el.querySelector('[data-act="depth+"]')!.addEventListener('click', () => this.post('depth', { delta: 0.05 }));
    this.setRightView('live');
  }

  public toggle() {
    if (this.open) this.close();
    else this.show();
  }

  public async show() {
    if (this.open) return;
    this.open = true;
    this.knownModelId = undefined;
    this.el.classList.add('visible', 'is-live');
    this.setStatus('Starting the camera...');
    this.setRightView(this.built ? 'model' : 'live');
    this.render();
    try {
      const resp = await fetch('/api/scan/start', { method: 'POST' });
      const res = await resp.json();
      if (!resp.ok || !res.ok) throw new Error(res.message || `HTTP ${resp.status}`);
    } catch (err: any) {
      this.setStatus(`Could not start the camera server: ${err?.message || err}. Is the app running via "npm run dev"?`, 'error');
      return;
    }
    this.connectStream(this.depthImg);
    if (this.rightView === 'live') this.connectStream(this.liveImg);
    this.poll();
  }

  /** Closes the panel and stops the scan server, which releases the camera. The built model is kept. */
  public close() {
    this.open = false;
    if (this.pollTimer !== null) window.clearTimeout(this.pollTimer);
    this.pollTimer = null;
    this.disconnectStream(this.depthImg);
    this.disconnectStream(this.liveImg);
    this.preview.stop();
    this.el.classList.remove('visible', 'is-live');
    fetch('/api/scan/stop', { method: 'POST' }).catch(() => undefined);
  }

  private setRightView(view: RightView) {
    this.rightView = view;
    this.el.querySelectorAll<HTMLButtonElement>('.camera-scan-tab').forEach((b) =>
      b.classList.toggle('is-active', b.dataset.right === view)
    );
    const live = view === 'live';
    const livePlaceholder = this.el.querySelector<HTMLElement>('[data-role="live-placeholder"]')!;
    if (live) {
      this.preview.stop();
      this.modelView.style.display = 'none';
      this.modelEmpty.style.display = 'none';
      livePlaceholder.style.display = 'flex';
      this.connectStream(this.liveImg);
    } else {
      this.disconnectStream(this.liveImg);
      livePlaceholder.style.display = 'none';
      const has = this.preview.hasModel();
      this.modelView.style.display = has ? 'block' : 'none';
      this.modelEmpty.style.display = has ? 'none' : 'flex';
      if (has && this.open) this.preview.start();
    }
    this.el.querySelector<HTMLElement>('[data-role="right-hint"]')!.textContent = live
      ? 'blue = table'
      : this.built
        ? `${this.built.sizeMm.map((v) => v.toFixed(0)).join(' × ')} mm · drag to rotate`
        : '';
  }

  private connectStream(img: HTMLImageElement) {
    if (!this.open) return;
    if (img === this.liveImg && this.rightView !== 'live') return;
    const name = img === this.depthImg ? 'depth' : 'cloud';
    // A fresh URL each time so the browser opens a new MJPEG connection.
    img.src = `/api/scan/cam/stream/${name}?t=${Date.now()}`;
  }

  private disconnectStream(img: HTMLImageElement) {
    img.removeAttribute('src');
    img.style.display = 'none';
  }

  private async post(path: string, body: unknown = {}) {
    try {
      const resp = await fetch(`/api/scan/cam/${path}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
      });
      const res = await resp.json();
      if (!resp.ok || res.ok === false) throw new Error(res.message || `HTTP ${resp.status}`);
      return res;
    } catch (err: any) {
      this.setStatus(err?.message || String(err), 'error');
      return null;
    }
  }

  private async build() {
    const res = await this.post('build');
    if (res) {
      this.setStatus(res.message);
      this.setRightView('live'); // watch the part while the frames are collected
    }
  }

  private poll = async () => {
    if (!this.open) return;
    try {
      const resp = await fetch('/api/scan/cam/status');
      const st = await resp.json();
      if (resp.ok) {
        this.lastStatus = st as ScanStatus;
        const model = this.lastStatus.model;
        // A model already on the server when the panel opened is not fetched again.
        if (this.knownModelId === undefined) this.knownModelId = model?.id ?? null;
        else if (model && model.id !== this.knownModelId) {
          this.knownModelId = model.id;
          await this.fetchModel(model);
        }
        if (!this.busy && st.message !== 'Model ready.') {
          this.setStatus(st.message, st.camera === 'error' ? 'error' : 'info');
        }
      } else {
        this.lastStatus = null;
        await this.showProcessState(st.message);
      }
    } catch {
      this.lastStatus = null;
    }
    this.render();
    if (this.open) this.pollTimer = window.setTimeout(this.poll, POLL_MS);
  };

  /** The server did not answer: say whether it is still starting or why it stopped. */
  private async showProcessState(fallback: string) {
    try {
      const proc = await (await fetch('/api/scan/proc')).json();
      if (!proc.running && proc.log?.length) {
        this.setStatus(`The camera server stopped:\n${proc.log.slice(-4).join('\n')}`, 'error');
        return;
      }
    } catch {
      // dev server restarting
    }
    this.setStatus(fallback || 'The camera server is starting...');
  }

  /** A new model was built: show it in the right view (it is not loaded for the gripper yet). */
  private async fetchModel(model: NonNullable<ScanStatus['model']>) {
    this.busy = true;
    try {
      const resp = await fetch(`/api/scan/cam/model.stl?id=${encodeURIComponent(model.id)}`);
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const buffer = await resp.arrayBuffer();
      this.preview.setModel(buffer);
      this.built = { id: model.id, file: model.file, sizeMm: model.sizeMm, buffer };
      this.setRightView('model');
      this.setStatus(
        `CAD model built: ${model.sizeMm.map((v) => v.toFixed(0)).join(' × ')} mm. ` +
          'Check it on the right, then press "Load model for gripper" (or rebuild).',
        'ok'
      );
    } catch (err: any) {
      this.setStatus(`Could not get the model: ${err?.message || err}`, 'error');
    } finally {
      this.busy = false;
    }
  }

  private async useModel() {
    if (!this.built || this.busy) return;
    this.busy = true;
    this.render();
    this.setStatus('Loading the model into the 3D view...');
    try {
      await this.events.onLoadModel(this.built.buffer.slice(0), this.built.file);
      this.loadedModelId = this.built.id;
      this.setStatus(
        `${this.built.file} is loaded in the 3D view. Click the model where finger 1 should grip, then finger 2. ` +
          'Close this panel (×) to see the whole view.',
        'ok'
      );
    } catch (err: any) {
      this.setStatus(`Could not load the model: ${err?.message || err}`, 'error');
    } finally {
      this.busy = false;
      this.render();
    }
  }

  private setStatus(text: string, kind: 'info' | 'ok' | 'error' = 'info') {
    this.statusEl.textContent = text;
    this.statusEl.className = `camera-scan-status ${kind === 'error' ? 'is-error' : kind === 'ok' ? 'is-ok' : ''}`;
  }

  private render() {
    const st = this.lastStatus;
    const streaming = st?.camera === 'streaming';
    this.buildBtn.disabled = !streaming || !!st?.building || this.busy || !st?.table || !st?.points;
    this.buildBtn.textContent = st?.building
      ? `Building ${st.building.frames}/${st.building.of}...`
      : this.built
        ? 'Rebuild CAD model'
        : 'Build CAD model';
    this.useBtn.disabled = !this.built || this.busy;
    this.useBtn.textContent =
      this.built && this.loadedModelId === this.built.id ? 'Loaded ✓ (load again)' : 'Load model for gripper';
    this.el.querySelector<HTMLElement>('[data-role="max-depth"]')!.textContent = `${(st?.maxDepthM ?? 0.8).toFixed(2)} m`;

    if (!st) {
      this.infoEl.textContent = '';
      return;
    }
    this.infoEl.textContent = [
      `${st.fps.toFixed(0)} fps`,
      `table ${st.table ? 'found' : 'NOT found'}`,
      st.points ? `object ${st.points} pts` : 'no object',
      st.distanceM !== null ? `${st.distanceM.toFixed(2)} m away` : '',
      st.sizeMm ? `~${st.sizeMm.map((v) => v.toFixed(0)).join('×')} mm` : ''
    ]
      .filter(Boolean)
      .join(' · ');
    this.buildBtn.title = !st.table
      ? 'The table plane is not found: the model is built on the table.'
      : !st.points
        ? 'No object detected on the table.'
        : 'Fuse 30 frames into a solid model. Keep the camera and the part still.';
  }
}
