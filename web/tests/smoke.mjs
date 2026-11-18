// Browser smoke test of the whole web flow against the real server:
//   load cad/main_part.step -> the fixture appears under it -> mark the two reference points -> Plan to Points
// Usage (from web/, after `npm run build`):  npm run smoke        [SMOKE_SIM=1 also runs Sim in a MuJoCo window]
// Starts `python -m server` on port 8766 (CELL_PYTHON / MUJOCO_PYTHON or python3) with the data in a temp folder.
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, '..', '..');
const port = 8766;
const base = `http://127.0.0.1:${port}`;
const python = process.env.CELL_PYTHON || process.env.MUJOCO_PYTHON || 'python3';
const dataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'cell-smoke-'));
const ref = JSON.parse(fs.readFileSync(path.join(repo, 'tests', 'fixtures', 'main_part_request.json'), 'utf8'));

const fail = (msg) => {
  console.error(`SMOKE FAIL: ${msg}`);
  process.exitCode = 1;
};

const server = spawn(python, ['-m', 'server'], {
  cwd: repo,
  env: { ...process.env, CELL_PORT: String(port), CELL_DATA_DIR: dataDir, PYTHONPATH: '' },
  stdio: ['ignore', 'pipe', 'pipe']
});
let serverLog = '';
server.stdout.on('data', (d) => (serverLog += d));
server.stderr.on('data', (d) => (serverLog += d));

const waitFor = async (fn, ms, what) => {
  const t0 = Date.now();
  while (Date.now() - t0 < ms) {
    try {
      if (await fn()) return;
    } catch {
      // not yet
    }
    await new Promise((r) => setTimeout(r, 250));
  }
  throw new Error(`timed out: ${what}`);
};

let browser;
try {
  await waitFor(async () => (await fetch(`${base}/api/health`)).ok, 20000, 'server start');
  browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1600, height: 1000 } });
  const errors = [];
  page.on('pageerror', (e) => errors.push(String(e)));
  await page.goto(base);

  // Load the part
  await page.setInputFiles('#cad-file-input', path.join(repo, 'cad', 'main_part.step'));
  await waitFor(() => page.evaluate(() => !!window.__viewer?.getModelRoot()), 60000, 'part loaded');
  await waitFor(
    () => page.evaluate(() => !!window.__viewer.getScene().getObjectByName('pick_fixture')),
    60000,
    'fixture shown'
  );
  console.log('ok: part loaded, fixture shown under it');

  // Mark the two reference points: project them (part frame, mm) to the screen and click there
  for (const [i, m] of ref.points.entries()) {
    // Look at the point straight along its surface normal (the two points are on opposite faces of a wall)
    const xy = await page.evaluate(({ p, n }) => {
      const v = window.__viewer;
      const cam = v.getCamera();
      const controls = v.getOrbitControls();
      const root = v.getModelRoot();
      root.updateMatrixWorld(true);
      const w = root.localToWorld(root.position.clone().set(p.x, p.y, p.z));
      const nw = w.clone().set(n.x, n.y, n.z).transformDirection(root.matrixWorld);
      cam.position.copy(w).addScaledVector(nw, 250);
      controls.target.copy(w);
      controls.update();
      cam.updateMatrixWorld(true);
      v.requestRender();
      const ndc = w.clone().project(cam);
      const r = v.getDomElement().getBoundingClientRect();
      return { x: r.left + ((ndc.x + 1) / 2) * r.width, y: r.top + ((1 - ndc.y) / 2) * r.height };
    }, { p: m.point, n: m.normal });
    await page.waitForTimeout(200);
    await page.mouse.click(xy.x, xy.y);
    await page.waitForTimeout(300);
    console.log(`ok: clicked point ${i + 1} at ${xy.x.toFixed(0)}, ${xy.y.toFixed(0)}`);
  }
  await waitFor(() => page.isEnabled('#auto-pick-calculate'), 10000, 'Plan to Points enabled (two points marked)');

  // Plan
  await page.click('#auto-pick-calculate');
  await waitFor(
    async () => (await page.getAttribute('#auto-pick-status', 'class'))?.match(/is-ok|is-error/),
    180000,
    'plan finished'
  );
  const status = await page.textContent('#auto-pick-status');
  if (!/complete motion/i.test(status)) fail(`planning: ${status}`);
  else console.log(`ok: ${status.trim().slice(0, 140)}`);

  if (process.env.SMOKE_SIM) {
    await page.click('#auto-pick-execute');
    await waitFor(
      async () => /grasp test done|placed in the tray|NOT placed|failed/i.test((await page.textContent('#auto-pick-status')) || ''),
      300000,
      'Sim finished'
    );
    console.log(`ok (Sim): ${(await page.textContent('#auto-pick-status')).trim().slice(0, 160)}`);
  }
  if (errors.length) fail(`page errors: ${errors.join(' | ')}`);
  if (!process.exitCode) console.log('SMOKE OK');
} catch (e) {
  fail(e.message);
  console.error(serverLog.split('\n').filter((l) => !l.includes('GET /api/mujoco/status')).slice(-15).join('\n'));
  try {
    console.error('panel status:', await browser?.contexts()[0]?.pages()[0]?.textContent('#auto-pick-status'));
  } catch {
    // no page
  }
  for (const run of fs.existsSync(path.join(dataDir, 'runs')) ? fs.readdirSync(path.join(dataDir, 'runs')) : []) {
    const dir = path.join(dataDir, 'runs', run);
    for (const f of ['status.json', 'log.txt']) {
      if (fs.existsSync(path.join(dir, f))) console.error(`${run}/${f}:`, fs.readFileSync(path.join(dir, f), 'utf8').slice(-800));
    }
  }
} finally {
  await browser?.close();
  server.kill('SIGTERM');
  fs.rmSync(dataDir, { recursive: true, force: true });
}
