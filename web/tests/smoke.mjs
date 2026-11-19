// Browser smoke test of the whole web flow against the real server:
//   load cad/main_part.step -> the fixture appears under it -> mark the two reference points -> Plan to Points
// Usage (from web/, after `npm run build`):  npm run smoke
//   SMOKE_SIM=1: also Sim, watched live in the page; SMOKE_REAL=1: Sim + Real Sim with the fake robot;
//   SMOKE_SHOTS=<dir>: screenshots of the live view
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
const touch = process.env.SMOKE_GRIP !== '1'; // default: the cell's scale, Touch mode. SMOKE_GRIP=1: main_part grip (needs the round fixture in the config)
const ref = touch
  ? { points: [{ point: { x: -28.4, y: -23.33, z: 80 }, normal: { x: 0, y: 1, z: 0 } }] }
  : JSON.parse(fs.readFileSync(path.join(repo, 'tests', 'fixtures', 'main_part_request.json'), 'utf8'));
const partFile = path.join(repo, 'cad', touch ? 'scale.step' : 'main_part.step');

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
  await page.setInputFiles('#cad-file-input', partFile);
  await waitFor(() => page.evaluate(() => !!window.__viewer?.getModelRoot()), 60000, 'part loaded');
  if (touch) {
    await page.click('.auto-pick-mode [data-mode="touch"]');
    console.log('ok: scale loaded (it is the fixture), Touch mode');
  } else {
    await waitFor(
      () => page.evaluate(() => !!window.__viewer.getScene().getObjectByName('pick_fixture')),
      60000,
      'fixture shown'
    );
    console.log('ok: part loaded, fixture shown under it');
  }

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
    async () => (await page.getAttribute('#cell-log .log-entry:last-child', 'class'))?.match(/is-ok|is-error/),
    180000,
    'plan finished'
  );
  const status = await page.textContent('#cell-log .log-entry:last-child');
  if (!/motion\(s\) planned/i.test(status)) fail(`planning: ${status}`);
  else console.log(`ok: ${status.trim().slice(0, 140)}`);

  const shots = process.env.SMOKE_SHOTS;
  if (shots) await page.screenshot({ path: path.join(shots, 'teach_planned.png') });
  const liveRun = async (button, what) => {
    await page.click(button);
    await waitFor(() => page.isVisible('.live-view-overlay.visible'), 10000, `${what}: live view shown`);
    await waitFor(
      () => page.evaluate(() => (window.__viewer.getScene().getObjectByName('live_cell')?.children.length ?? 0) > 10),
      120000,
      `${what}: live cell built`
    );
    const pose = () =>
      page.evaluate(() => {
        const cell = window.__viewer.getScene().getObjectByName('live_cell');
        return cell.children.filter((c) => c.type === 'Group').map((g) => g.position.toArray().map((v) => v.toFixed(4)).join()).join('|');
      });
    // Sim first settles the part and holds still for the preview (a few seconds): wait for the arm to move
    const p0 = await pose();
    try {
      await waitFor(async () => (await pose()) !== p0, 30000, 'movement');
      console.log(`ok (${what}): live cell shown and moving`);
    } catch {
      fail(`${what}: the live cell did not move in 30 s`);
    }
    await page.waitForTimeout(3000);
    if (shots) await page.screenshot({ path: path.join(shots, `${what}_live.png`) });
    await waitFor(
      async () => (await page.getAttribute('#cell-log .log-entry:last-child', 'class'))?.match(/is-ok|is-error/) &&
        !(await page.isVisible('.live-view-overlay.is-live')),
      300000,
      `${what} finished`
    );
    if (shots) await page.screenshot({ path: path.join(shots, `${what}_end.png`) });
    console.log(`ok (${what}): ${(await page.textContent('#cell-log .log-entry:last-child')).trim().slice(0, 160)}`);
    if (!/is-ok/.test(await page.getAttribute('#cell-log .log-entry:last-child', 'class'))) fail(`${what} did not succeed`);
  };
  if (process.env.SMOKE_SIM || process.env.SMOKE_REAL) await liveRun('#auto-pick-execute', 'sim');
  if (process.env.SMOKE_REAL) {
    page.on('dialog', (d) => d.accept());
    await page.fill('#auto-pick-real-ip', 'fake');
    await page.dispatchEvent('#auto-pick-real-ip', 'change');
    await waitFor(() => page.isEnabled('#auto-pick-real-run'), 10000, 'Real Sim enabled');
    await liveRun('#auto-pick-real-run', 'real');
  }
  if (errors.length) fail(`page errors: ${errors.join(' | ')}`);
  if (!process.exitCode) console.log('SMOKE OK');
} catch (e) {
  fail(e.message);
  console.error(serverLog.split('\n').filter((l) => !l.includes('GET /api/mujoco/status')).slice(-15).join('\n'));
  try {
    console.error('panel status:', await browser?.contexts()[0]?.pages()[0]?.textContent('#cell-log .log-entry:last-child'));
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
