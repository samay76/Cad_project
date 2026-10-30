/**
 * Floating panel in the 3D viewport that shows the MuJoCo inspection camera:
 * the live feed while a simulation runs, and the inspection images when they are ready.
 * Images are served by the dev-server bridge (vite-plugins/mujocoBridge.ts, /api/mujoco/file).
 */
export interface InspectionImage {
  label: string;
  file: string;
  slip_mm?: number;
}

export class CameraFeedPanel {
  private el: HTMLElement;
  private img: HTMLImageElement;
  private placeholder: HTMLElement;
  private caption: HTMLElement;
  private gallery: HTMLElement;
  private runId: string | null = null;
  private live = false;
  private timer: number | null = null;
  private shownImages = 0;

  constructor(parent: HTMLElement) {
    this.el = document.createElement('div');
    this.el.className = 'camera-feed-panel';
    this.el.innerHTML = `
      <div class="camera-feed-header">
        <span class="camera-feed-live-dot"></span>
        <span class="camera-feed-title">Inspection camera (MuJoCo)</span>
        <button class="camera-feed-close" title="Hide camera panel">&times;</button>
      </div>
      <div class="camera-feed-view">
        <img class="camera-feed-img" alt="Inspection camera feed" />
        <div class="camera-feed-placeholder">Waiting for the camera...</div>
      </div>
      <div class="camera-feed-caption"></div>
      <div class="camera-feed-gallery"></div>
    `;
    parent.appendChild(this.el);
    this.img = this.el.querySelector('.camera-feed-img')!;
    this.placeholder = this.el.querySelector('.camera-feed-placeholder')!;
    this.caption = this.el.querySelector('.camera-feed-caption')!;
    this.gallery = this.el.querySelector('.camera-feed-gallery')!;

    this.el.querySelector('.camera-feed-close')!.addEventListener('click', () => this.hide());
    // Load the next frame only after the previous one arrived (no request pile-up).
    this.img.addEventListener('load', () => {
      this.placeholder.style.display = 'none';
      this.img.style.display = 'block';
      this.scheduleNext(150);
    });
    this.img.addEventListener('error', () => this.scheduleNext(600));
  }

  private url(name: string) {
    return `/api/mujoco/file?id=${encodeURIComponent(this.runId!)}&name=${encodeURIComponent(name)}`;
  }

  private scheduleNext(ms: number) {
    if (this.timer !== null) window.clearTimeout(this.timer);
    if (!this.live || !this.runId) return;
    this.timer = window.setTimeout(() => {
      this.img.src = `${this.url('feed.jpg')}&t=${Date.now()}`;
    }, ms);
  }

  /** Start showing the live feed of a new simulation run. */
  public start(runId: string) {
    this.runId = runId;
    this.live = true;
    this.shownImages = 0;
    this.gallery.innerHTML = '';
    this.caption.textContent = '';
    this.img.style.display = 'none';
    this.placeholder.style.display = 'flex';
    this.placeholder.textContent = 'Waiting for the camera...';
    this.el.classList.add('visible', 'is-live');
    this.scheduleNext(0);
  }

  public setCaption(text: string) {
    this.caption.textContent = text;
  }

  /** Inspection stills (label, file, slip) reported by the simulator so far. */
  public setImages(images: InspectionImage[]) {
    if (!this.runId || images.length === this.shownImages) return;
    this.shownImages = images.length;
    this.gallery.innerHTML = images
      .map((im, i) => {
        const src = this.url(im.file);
        const slip = im.slip_mm !== undefined ? ` · slip ${im.slip_mm} mm` : '';
        return `
          <a class="camera-feed-thumb" href="${src}" target="_blank" rel="noopener" title="${im.label}${slip} (open full size)">
            <img src="${src}&t=${Date.now()}" alt="${im.label}" data-src="${src}" data-tries="0" />
            <span>${i + 1}. ${im.label}${slip}</span>
          </a>`;
      })
      .join('');
    // A still can be listed a moment before the camera process has written it: retry briefly.
    this.gallery.querySelectorAll<HTMLImageElement>('img').forEach((img) => {
      img.addEventListener('error', () => {
        const tries = Number(img.dataset.tries || 0);
        if (tries < 10) {
          img.dataset.tries = String(tries + 1);
          window.setTimeout(() => (img.src = `${img.dataset.src}&t=${Date.now()}`), 500);
        }
      });
    });
  }

  /** Simulation finished: keep refreshing while the MuJoCo window is still open (feed keeps running). */
  public finish(message: string) {
    this.setCaption(message);
    this.el.classList.remove('is-live');
  }

  public hide() {
    this.live = false;
    if (this.timer !== null) window.clearTimeout(this.timer);
    this.timer = null;
    this.el.classList.remove('visible', 'is-live');
  }
}
