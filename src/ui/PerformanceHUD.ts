/**
 * Performance HUD
 * Displays real-time FPS and collision check duration in milliseconds.
 * Toggled on/off with the 'P' key.
 */
export class PerformanceHUD {
  public static instance: PerformanceHUD | null = null;

  private container: HTMLElement;
  private fpsElement: HTMLElement | null = null;
  private colElement: HTMLElement | null = null;
  private isVisible = true;

  // Frame timing
  private lastFrameTime = performance.now();
  private frameTimes: number[] = [];
  private maxFrameSamples = 30;

  // Collision timing
  private lastCollisionDuration = 0;
  private maxAllowedColTimeMs = 8.0;

  private boundKeyDown: (e: KeyboardEvent) => void;

  constructor(parent: HTMLElement) {
    PerformanceHUD.instance = this;
    this.container = document.createElement('div');
    this.container.className = 'perf-hud';
    this.container.innerHTML = `
      <div class="perf-title">PERF HUD <span class="perf-key">[P]</span></div>
      <div class="perf-row">
        <span class="perf-label">FPS:</span>
        <span class="perf-val" id="perf-fps">60</span>
      </div>
      <div class="perf-row">
        <span class="perf-label">Collision:</span>
        <span class="perf-val" id="perf-col">0.0 ms</span>
      </div>
    `;

    parent.appendChild(this.container);
    this.fpsElement = this.container.querySelector('#perf-fps');
    this.colElement = this.container.querySelector('#perf-col');

    this.boundKeyDown = (e: KeyboardEvent) => {
      // Don't trigger if user is typing in an input
      if (['INPUT', 'TEXTAREA', 'SELECT'].includes((e.target as HTMLElement)?.tagName)) {
        return;
      }
      if (e.key === 'p' || e.key === 'P') {
        this.toggle();
      }
    };
    window.addEventListener('keydown', this.boundKeyDown);
  }

  public toggle() {
    this.isVisible = !this.isVisible;
    this.container.style.display = this.isVisible ? 'flex' : 'none';
  }

  public setVisible(visible: boolean) {
    this.isVisible = visible;
    this.container.style.display = this.isVisible ? 'flex' : 'none';
  }

  /**
   * Called on every rendered frame to update FPS counter
   */
  public recordFrame(now = performance.now()) {
    const delta = now - this.lastFrameTime;
    this.lastFrameTime = now;

    if (delta > 0) {
      this.frameTimes.push(1000 / delta);
      if (this.frameTimes.length > this.maxFrameSamples) {
        this.frameTimes.shift();
      }
    }

    if (this.fpsElement && this.isVisible && this.frameTimes.length > 0) {
      const avgFps = Math.round(
        this.frameTimes.reduce((a, b) => a + b, 0) / this.frameTimes.length
      );
      this.fpsElement.textContent = `${avgFps}`;
      if (avgFps < 45) {
        this.fpsElement.style.color = '#ef4444';
      } else if (avgFps < 55) {
        this.fpsElement.style.color = '#f59e0b';
      } else {
        this.fpsElement.style.color = '#10b981';
      }
    }
  }

  /**
   * Records collision check duration and warns if > 8ms
   */
  public recordCollisionTime(durationMs: number) {
    this.lastCollisionDuration = durationMs;

    if (this.colElement && this.isVisible) {
      this.colElement.textContent = `${durationMs.toFixed(2)} ms`;
      if (durationMs > this.maxAllowedColTimeMs) {
        this.colElement.style.color = '#ef4444';
      } else if (durationMs > 4.0) {
        this.colElement.style.color = '#f59e0b';
      } else {
        this.colElement.style.color = '#10b981';
      }
    }

    if (durationMs > this.maxAllowedColTimeMs) {
      console.warn(
        `[PERF WARNING] Collision check took ${durationMs.toFixed(2)} ms (exceeds ${this.maxAllowedColTimeMs} ms target)`
      );
    }
  }

  public dispose() {
    window.removeEventListener('keydown', this.boundKeyDown);
    if (this.container.parentElement) {
      this.container.parentElement.removeChild(this.container);
    }
  }
}
