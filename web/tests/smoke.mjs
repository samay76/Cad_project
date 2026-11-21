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
// The cell's yoke on Fixture2: the two faces of the -x ear (the same points as tests/test_planning.py)
const ref = {
  points: [
    { point: { x: -20.6, y: 0.248, z: 57.198 }, normal: { x: -1, y: 0, z: 0 } },
    { point: { x: -14.625, y: -0.639, z: 57.785 }, normal: { x: 1, y: 0, z: 0 } }
  ]
};
const partFile = path.join(repo, 'tests', 'fixtures', 'cad', 'Part3.step');

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
  const shotsEarly = process.env.SMOKE_SHOTS;
  await waitFor(() => page.evaluate(() => (() => { let n = 0; window.__viewer?.getScene().getObjectByName('live_cell')?.traverse((o) => { if (o.isMesh) n++; }); return n; })() > 10), 180000, 'workspace at start');
  await page.waitForTimeout(1500);
  if (shotsEarly) await page.screenshot({ path: path.join(shotsEarly, 'app_start.png') });
  console.log('ok: the app opens in the workspace');
  if (!(await page.isVisible('#op-stop')) || !(await page.isEnabled('#op-stop'))) fail('Stop robot is not visible and enabled');
  // Going ahead without a part is refused with a reason
  await page.click('.op-step[data-step="3"]');
  if (!/Load a part/.test((await page.textContent('#op-stepper')) || '')) fail('clicking a locked step did not say why');
  else console.log('ok: a locked step says why');

  // Fixture library: add fixture F1, add the part to it, use it
  await page.click('#op-add-fixture');
  await page.click('.op-ask button[type=submit]'); // without the fixture CAD: refused
  if (!/fixture CAD file first/.test((await page.textContent('.op-ask-error')) || '')) fail('a fixture without its CAD file was not refused');
  await page.setInputFiles('.op-ask input[type=file]', path.join(repo, 'tests', 'fixtures', 'cad', 'Fixture2.step'));
  if ((await page.inputValue('.op-ask input[name=name]')) !== 'Fixture2') fail('fixture name not taken from the file');
  await page.fill('.op-ask input[name=name]', 'F1');
  await page.click('.op-ask button[type=submit]');
  await waitFor(() => page.isVisible('.op-fixture-window'), 5000, 'fixture window');
  if (!/Fixture2\.step/.test((await page.textContent('.op-fixture-file')) || '')) fail('fixture CAD not shown in the fixture window');
  await page.setInputFiles('.op-fixture-window input[data-parts]', partFile);
  await waitFor(() => page.isVisible('.op-part-row [data-act="use"]'), 30000, 'part added to F1');
  await page.setInputFiles('.op-fixture-window input[data-parts]', path.join(repo, 'README.md'));
  await waitFor(async () => /Only STEP files can be added/.test((await page.textContent('#op-status-row')) || ''), 5000, 'non-STEP refused');
  console.log('ok: fixture F1 created with its CAD (Fixture2.step), Part3 added, a non-STEP file refused');
  await page.click('.op-part-row [data-act="use"]');
  await waitFor(() => page.evaluate(() => !!window.__viewer?.getModelRoot()), 60000, 'part loaded');
  if (!/Fixture F1 · Part3/.test((await page.textContent('#op-job')) || '')) fail('header does not show the fixture and part');
  const statusHeight = await page.evaluate(() => document.getElementById('op-activity').getBoundingClientRect().height);
  if (statusHeight > 46) fail(`status bar is ${statusHeight}px tall`);
  await waitFor(
    () => page.evaluate(() => !!window.__viewer.getScene().getObjectByName('pick_fixture')),
    60000,
    'fixture shown'
  );
  console.log('ok: part loaded, fixture shown under it');
  if (shotsEarly) await page.screenshot({ path: path.join(shotsEarly, 'step1_loaded.png') });
  const pointCount = () => page.evaluate(() => window.__autoPick.points.length);

  // Clicks a reference point (part frame, mm) on the screen
  const clickPoint = async (m) => {
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
    return xy;
  };
  // Mark the two reference points: project them (part frame, mm) to the screen and click there.
  // First in the Load part step: teach mode is off, so nothing may be marked.
  for (const [pass, m] of [ref.points[0], ...ref.points].entries()) {
    const i = pass - 1;
    if (pass === 1) {
      if ((await pointCount()) !== 0) fail('a click in the Load part step marked a point');
      else console.log('ok: a click on the part outside Teach grip marks nothing');
      await page.click('#op-next-1');
      await waitFor(() => page.isVisible('.auto-pick-mode'), 5000, 'Teach grip step');
      if (!/finger 1/.test((await page.textContent('.op-teach-banner')) || '')) fail('teach banner missing');
    }
    const xy = await clickPoint(m);
    if (pass) console.log(`ok: clicked point ${i + 1} at ${xy.x.toFixed(0)}, ${xy.y.toFixed(0)}`);
  }
  await waitFor(() => page.isEnabled('#auto-pick-calculate'), 10000, 'Find best grip enabled (two points marked)');
  // Reset points (Teach grip): both points go, then they come back (same spots, restored for the rest of the test)
  page.once('dialog', (d) => d.accept());
  await page.click('#auto-pick-clear');
  await waitFor(async () => (await pointCount()) === 0, 5000, 'Reset points removed the points');
  if (await page.isEnabled('#auto-pick-calculate')) fail('Find best grip enabled after Reset points');
  if (await page.isEnabled('#auto-pick-clear')) fail('Reset points enabled with no points');
  console.log('ok: Reset points clears the grip points in Teach grip');
  for (const m of ref.points) await clickPoint(m);
  await waitFor(() => page.isEnabled('#auto-pick-calculate'), 10000, 'Find best grip enabled (points back)');
  if (shotsEarly) await page.screenshot({ path: path.join(shotsEarly, 'step2_points.png') });

  // Plan
  await page.click('#auto-pick-calculate');
  // planning, then the arm drives to the pick point by itself (approach) and stays there
  await waitFor(
    async () => /arm at the pick point|error|cannot|could not/i.test((await page.textContent('#cell-log')) || '') &&
      (await page.getAttribute('#cell-log .log-entry:last-child', 'class'))?.match(/is-ok|is-error/),
    240000,
    'plan + arm at the pick point'
  );
  const log = (await page.textContent('#cell-log')) || '';
  const req = fs.readdirSync(path.join(dataDir, 'runs')).map((r) => path.join(dataDir, 'runs', r, 'request.json')).find((f) => fs.existsSync(f));
  const fixtureFile = req && JSON.parse(fs.readFileSync(req, 'utf8')).fixtureFile;
  if (!fixtureFile || !fixtureFile.includes(path.join('library', 'fixtures'))) fail(`the plan does not use the library fixture: ${fixtureFile}`);
  else console.log('ok: the plan uses the library fixture CAD');
  const planned = log.match(/\d+ (touch|complete) motion\(s\) planned[^.]*/i)?.[0];
  if (!planned) fail(`planning: ${log.slice(-300)}`);
  else console.log(`ok: ${planned}`);
  if (!/The arm at the pick point/i.test(log)) fail('the arm did not go to the pick point');
  else console.log('ok: the arm is at the pick point after planning (fingers closed)');
  if (shotsEarly) await page.screenshot({ path: path.join(shotsEarly, 'step3_planned.png') });
  await waitFor(() => page.isVisible('#auto-pick-execute'), 5000, 'the Simulate step opens after a grip is found');
  if (await page.isEnabled('#op-next-3')) fail('Next: run on robot is enabled before a simulation');
  else console.log('ok: Simulate step shown; Run on robot locked until the simulation ends');
  const before = await page.evaluate(() => JSON.stringify(window.__autoPick.points));
  await page.evaluate(() => window.__autoPick.markPoint({ fromSurfaceClick: true, point: { x: 1, y: 2, z: 3 }, normal: { x: 0, y: 0, z: 1 } }));
  if ((await page.evaluate(() => JSON.stringify(window.__autoPick.points))) !== before) fail('a click in the Simulate step changed the points');
  else console.log('ok: a click on the part in the Simulate step does not change the points');

  const shots = process.env.SMOKE_SHOTS;
  if (shots) await page.screenshot({ path: path.join(shots, 'teach_planned.png') });
  const liveRun = async (button, what) => {
    await page.click(button);
    await waitFor(() => page.isVisible('.live-view-overlay.visible'), 10000, `${what}: live view shown`);
    await waitFor(
      () => page.evaluate(() => (() => { let n = 0; window.__viewer.getScene().getObjectByName('live_cell')?.traverse((o) => { if (o.isMesh) n++; }); return n; })() > 10),
      120000,
      `${what}: live cell built`
    );
    const pose = () =>
      page.evaluate(() => {
        const cell = window.__viewer.getScene().getObjectByName('live_cell');
        const groups = []; cell.traverse((c) => { if (c.type === 'Group' && c !== cell && c.name !== 'cell_room') groups.push(c); }); return groups.map((g) => g.position.toArray().map((v) => v.toFixed(4)).join()).join('|');
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
    await page.click('#op-settings-btn');
    await page.fill('#auto-pick-real-ip', 'fake');
    await page.click('#op-drawer-close');
    await waitFor(() => page.isEnabled('#op-next-3'), 10000, 'Next: run on robot enabled');
    await page.click('#op-next-3');
    if (await page.isEnabled('#auto-pick-real-run')) fail('Start robot is enabled before the safety checks');
    await page.check('#op-check-area');
    if (await page.isEnabled('#auto-pick-real-run')) fail('Start robot is enabled with one safety check');
    await page.check('#op-check-part');
    await waitFor(() => page.isEnabled('#auto-pick-real-run'), 5000, 'Start robot enabled');
    if (shots) await page.screenshot({ path: path.join(shots, 'step4_ready.png') });
    await liveRun('#auto-pick-real-run', 'real');
    // After the run: one button back to teaching for the next part
    await waitFor(() => page.isVisible('#op-teach-next'), 15000, '"Teach next part" after the run').catch(async (e) => { console.log('PANEL:', (await page.innerText('#auto-pick-mount')).replace(/\s+/g, ' ')); console.log('STEP:', await page.evaluate(() => document.querySelector('#op-stepper')?.innerText)); throw e; });
    if (shots) await page.screenshot({ path: path.join(shots, 'step4_done.png') });
    await page.click('#op-teach-next');
    await waitFor(() => page.isVisible('#op-fixtures'), 5000, 'step 1 (fixture library) after "Teach next part"');
    console.log('ok: "Teach next part" goes back to choosing a part');
  }
  // The library survives a reload, with the taught grip points saved on the part
  await page.reload();
  await waitFor(() => page.isVisible('.op-fixture-card'), 30000, 'fixtures after reload');
  const card = (await page.textContent('.op-fixture-card')) || '';
  if (!/F1/.test(card) || !/1 part/.test(card)) fail(`fixture card after reload: ${card}`);
  await page.click('.op-fixture-card');
  if (!(await page.isVisible('.op-part-row .op-tag.is-ok'))) fail('grip points were not saved with the part');
  else console.log('ok: after a reload F1 still has Part3, with its grip points saved');
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
