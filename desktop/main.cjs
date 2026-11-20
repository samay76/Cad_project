// Pick Cell desktop app: starts the cell server (python -m server), then shows the app in its own window.
//
//   1. a loading screen appears at once
//   2. the interface is rebuilt only if web/src changed since the last build (npm run build in web/)
//   3. the server is started (or a running one on the same port is reused, e.g. the systemd service)
//   4. the main window opens when its first frame is drawn (no white flash), maximised
//   5. closing the window stops the server; a running job (simulation or the real robot) is stopped first
//
// Settings: the same .env as the server (CELL_PYTHON, CELL_PORT, ...). Logs: var/desktop/server.log.
const { app, BrowserWindow, Menu, dialog, shell, nativeImage } = require('electron');
const { spawn, spawnSync } = require('child_process');
const fs = require('fs');
const http = require('http');
const net = require('net');
const os = require('os');
const path = require('path');

const REPO = path.resolve(__dirname, '..');
const WEB = path.join(REPO, 'web');
const ICON = path.join(__dirname, 'icon.png');
const LOG_DIR = path.join(REPO, 'var', 'desktop');
const BG = '#f1f5f9'; // the app's background (web/src/styles.css --bg-app)

// Smooth 3D: use the GPU even where Chromium's blocklist is cautious (Intel iGPU, Wayland).
app.commandLine.appendSwitch('ignore-gpu-blocklist');
app.commandLine.appendSwitch('enable-gpu-rasterization');
app.commandLine.appendSwitch('enable-zero-copy');
app.commandLine.appendSwitch('ozone-platform-hint', 'auto');
app.setName('Pick Cell');
if (process.platform === 'linux') app.setDesktopName('pick-cell.desktop'); // taskbar icon = the launcher's

// ---------------------------------------------------------------- settings
function readDotenv() {
  const out = {};
  try {
    for (const raw of fs.readFileSync(path.join(REPO, '.env'), 'utf8').split('\n')) {
      const line = raw.trim();
      if (!line || line.startsWith('#') || !line.includes('=')) continue;
      const i = line.indexOf('=');
      out[line.slice(0, i).trim()] = line.slice(i + 1).trim().replace(/^["']|["']$/g, '');
    }
  } catch { /* no .env: defaults */ }
  return out;
}
const DOTENV = readDotenv();
const setting = (key, fallback) => process.env[key] || DOTENV[key] || fallback;

function findPython() {
  const home = os.homedir();
  const candidates = [
    setting('CELL_PYTHON'),
    ...['miniforge3', 'mambaforge', 'miniconda3', 'anaconda3'].flatMap((d) =>
      ['pick-cell', 'mujoco_cad'].map((e) => path.join(home, d, 'envs', e, 'bin', 'python'))),
  ].filter(Boolean);
  for (const p of candidates) {
    if (!fs.existsSync(p)) continue;
    const r = spawnSync(p, ['-c', 'import fastapi, uvicorn, mujoco'], { env: { ...process.env, PYTHONPATH: '' } });
    if (r.status === 0) return p;
  }
  return null;
}

// ---------------------------------------------------------------- helpers
function getJson(url, timeoutMs = 1500) {
  return new Promise((resolve) => {
    const req = http.get(url, { timeout: timeoutMs }, (res) => {
      let body = '';
      res.on('data', (c) => (body += c));
      res.on('end', () => {
        try { resolve(res.statusCode === 200 ? JSON.parse(body) : null); } catch { resolve(null); }
      });
    });
    req.on('timeout', () => req.destroy());
    req.on('error', () => resolve(null));
  });
}

function portFree(port) {
  return new Promise((resolve) => {
    const s = net.createServer();
    s.once('error', () => resolve(false));
    s.once('listening', () => s.close(() => resolve(true)));
    s.listen(port, '127.0.0.1');
  });
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function newestMtime(p) {
  if (!fs.existsSync(p)) return 0;
  const st = fs.statSync(p);
  if (!st.isDirectory()) return st.mtimeMs;
  let m = 0;
  for (const name of fs.readdirSync(p)) m = Math.max(m, newestMtime(path.join(p, name)));
  return m;
}

function webNeedsBuild() {
  const built = path.join(WEB, 'dist', 'index.html');
  if (!fs.existsSync(built)) return true;
  const sources = ['src', 'public', 'index.html', 'package.json', 'vite.config.ts', 'tsconfig.json']
    .map((p) => newestMtime(path.join(WEB, p)));
  return Math.max(...sources) > fs.statSync(built).mtimeMs;
}

function run(cmd, args, opts) {
  return new Promise((resolve) => {
    const p = spawn(cmd, args, { ...opts, stdio: ['ignore', 'pipe', 'pipe'] });
    let out = '';
    p.stdout.on('data', (d) => (out += d));
    p.stderr.on('data', (d) => (out += d));
    p.on('error', (e) => resolve({ code: -1, out: String(e) }));
    p.on('close', (code) => resolve({ code, out }));
  });
}

// ---------------------------------------------------------------- loading screen
let splash = null;
function showSplash() {
  splash = new BrowserWindow({
    width: 460, height: 260, frame: false, resizable: false, movable: true, show: false, center: true,
    backgroundColor: '#0f172a', skipTaskbar: false, title: 'Pick Cell', icon: ICON,
    webPreferences: { contextIsolation: true, sandbox: true },
  });
  splash.loadFile(path.join(__dirname, 'splash.html'));
  splash.once('ready-to-show', () => splash && splash.show());
}
function status(text) {
  console.log(`[desktop] ${text}`);
  if (splash && !splash.isDestroyed()) {
    splash.webContents.executeJavaScript(`window.setStatus && window.setStatus(${JSON.stringify(text)})`).catch(() => {});
  }
}

// ---------------------------------------------------------------- server
let server = null;       // the child process we started (null when we reuse a running server)
let serverLog = null;
let stopping = false;

function tailLog(n = 25) {
  try {
    return fs.readFileSync(path.join(LOG_DIR, 'server.log'), 'utf8').trim().split('\n').slice(-n).join('\n');
  } catch { return ''; }
}

async function startServer() {
  const wanted = parseInt(setting('CELL_PORT', '8000'), 10);
  // A cell server already running here (e.g. the systemd service): use it rather than start a second one.
  const health = await getJson(`http://127.0.0.1:${wanted}/api/health`);
  if (health && health.ok) return wanted;

  let port = wanted;
  if (!(await portFree(port))) {
    port = 0;
    for (let p = 8765; p < 8800; p++) if (await portFree(p)) { port = p; break; }
    if (!port) throw new Error(`Port ${wanted} is in use by another program and no free port was found.`);
  }

  const python = findPython();
  if (!python) {
    throw new Error('No Python environment with the cell software was found.\n'
      + 'Set CELL_PYTHON in .env (e.g. ~/miniforge3/envs/pick-cell/bin/python) or run deploy/install.sh.');
  }
  fs.mkdirSync(LOG_DIR, { recursive: true });
  serverLog = fs.createWriteStream(path.join(LOG_DIR, 'server.log'), { flags: 'w' });
  const env = {
    ...process.env, PYTHONPATH: '', PYTHONUNBUFFERED: '1', CELL_PORT: String(port), CELL_HOST: '127.0.0.1',
    CELL_PYTHON: setting('CELL_PYTHON', python),
  };
  server = spawn(python, ['-m', 'server'], { cwd: REPO, env, stdio: ['ignore', 'pipe', 'pipe'] });
  server.stdout.pipe(serverLog, { end: false });
  server.stderr.pipe(serverLog, { end: false });
  let exited = null;
  server.on('exit', (code, sig) => {
    exited = { code, sig };
    server = null;
    if (!stopping && mainWindow) {
      dialog.showErrorBox('Pick Cell', `The cell server stopped (${sig || 'exit code ' + code}).\n\n${tailLog()}`);
      app.quit();
    }
  });

  const t0 = Date.now();
  while (Date.now() - t0 < 90_000) {
    if (exited) throw new Error(`The cell server did not start (exit code ${exited.code}).\n\n${tailLog()}`);
    const h = await getJson(`http://127.0.0.1:${port}/api/health`, 1000);
    if (h && h.ok) return port;
    await sleep(250);
  }
  throw new Error(`The cell server did not answer within 90 s.\n\n${tailLog()}`);
}

function stopServer() {
  return new Promise((resolve) => {
    if (!server) return resolve();
    const child = server;
    const kill = setTimeout(() => { try { child.kill('SIGKILL'); } catch { /* gone */ } }, 20_000);
    child.once('exit', () => { clearTimeout(kill); resolve(); });
    child.kill('SIGTERM'); // the server stops a running job first (real mode: sends `stop` to the robot)
  });
}

// ---------------------------------------------------------------- main window
let mainWindow = null;

function buildMenu() {
  const send = (js) => mainWindow && mainWindow.webContents.executeJavaScript(js, true).catch(() => {});
  const template = [
    {
      label: 'File',
      submenu: [
        { label: 'Open Part…', accelerator: 'CmdOrCtrl+O', click: () => send("document.getElementById('cad-file-input')?.click()") },
        { type: 'separator' },
        { label: 'Open Logs Folder', click: () => shell.openPath(LOG_DIR) },
        { type: 'separator' },
        { role: 'quit', label: 'Quit', accelerator: 'CmdOrCtrl+Q' },
      ],
    },
    {
      label: 'View',
      submenu: [
        { role: 'reload', label: 'Reload' },
        { type: 'separator' },
        { role: 'resetZoom' }, { role: 'zoomIn' }, { role: 'zoomOut' },
        { type: 'separator' },
        { role: 'togglefullscreen' },
        { role: 'toggleDevTools', label: 'Developer Tools' },
      ],
    },
    {
      label: 'Help',
      submenu: [
        { label: 'README', click: () => shell.openPath(path.join(REPO, 'README.md')) },
        { label: 'About Pick Cell', click: () => dialog.showMessageBox(mainWindow, {
          type: 'info', title: 'Pick Cell', icon: ICON,
          message: 'Pick Cell', detail: `EC66 + Robotiq 2F-85 pick & place\nVersion ${app.getVersion()} · Electron ${process.versions.electron}`,
        }) },
      ],
    },
  ];
  Menu.setApplicationMenu(Menu.buildFromTemplate(template));
}

function openMain(port) {
  const origin = `http://127.0.0.1:${port}`;
  mainWindow = new BrowserWindow({
    width: 1600, height: 960, minWidth: 1100, minHeight: 700, show: false,
    backgroundColor: BG, title: 'Pick Cell', icon: ICON, autoHideMenuBar: false,
    webPreferences: { contextIsolation: true, sandbox: true, nodeIntegration: false, spellcheck: false },
  });
  buildMenu();
  const wc = mainWindow.webContents;
  // Stay inside the app: other links open in the normal browser.
  wc.setWindowOpenHandler(({ url }) => { shell.openExternal(url); return { action: 'deny' }; });
  wc.on('will-navigate', (e, url) => { if (!url.startsWith(origin)) { e.preventDefault(); shell.openExternal(url); } });
  wc.on('render-process-gone', (_e, d) => {
    if (d.reason === 'clean-exit') return;
    const r = dialog.showMessageBoxSync(mainWindow, {
      type: 'error', title: 'Pick Cell', message: 'The window stopped unexpectedly.', detail: `Reason: ${d.reason}`,
      buttons: ['Reload', 'Quit'], defaultId: 0,
    });
    if (r === 0) wc.reload(); else app.quit();
  });
  wc.on('page-title-updated', (e) => e.preventDefault());

  mainWindow.once('ready-to-show', () => {
    mainWindow.maximize();
    mainWindow.show();
    if (splash && !splash.isDestroyed()) splash.close();
    splash = null;
    // Self-check (PICK_CELL_SELFTEST=<png>): screenshot after the workspace has loaded, report GPU, quit.
    const shot = process.env.PICK_CELL_SELFTEST;
    if (shot) setTimeout(async () => {
      const img = await mainWindow.webContents.capturePage();
      fs.writeFileSync(shot, img.toPNG());
      const gpu = app.getGPUFeatureStatus();
      const fps = await wc.executeJavaScript(`new Promise(r => { let n = 0; const t0 = performance.now();
        const f = () => { n++; performance.now() - t0 < 2000 ? requestAnimationFrame(f) : r(n / 2); }; requestAnimationFrame(f); })`);
      console.log(`[selftest] webgl=${gpu.webgl} webgl2=${gpu.webgl2} gpu_compositing=${gpu.gpu_compositing} raf_fps=${fps}`);
      quitApp();
    }, 15_000);
  });
  mainWindow.on('close', (e) => {
    if (stopping) return;
    e.preventDefault();
    confirmClose();
  });
  mainWindow.loadURL(origin + '/');
}

async function confirmClose() {
  const port = mainWindow.webContents.getURL().match(/:(\d+)\//)?.[1];
  const h = port ? await getJson(`http://127.0.0.1:${port}/api/health`) : null;
  if (h && h.running && server) {
    const r = dialog.showMessageBoxSync(mainWindow, {
      type: 'warning', title: 'Pick Cell', buttons: ['Stop and quit', 'Cancel'], defaultId: 1, cancelId: 1,
      message: 'A job is running.', detail: 'Quitting stops it (in Real Sim the robot is sent `stop`).',
    });
    if (r !== 0) return;
  }
  quitApp();
}

async function quitApp() {
  if (stopping) return;
  stopping = true;
  if (mainWindow && !mainWindow.isDestroyed()) mainWindow.hide();
  await stopServer();
  app.exit(0);
}

// ---------------------------------------------------------------- start
async function boot() {
  showSplash();
  try {
    if (webNeedsBuild()) {
      status('Updating the interface…');
      const r = await run('npm', ['run', 'build'], { cwd: WEB, env: process.env });
      if (r.code !== 0) {
        if (!fs.existsSync(path.join(WEB, 'dist', 'index.html'))) throw new Error(`Building the interface failed:\n\n${r.out.slice(-2000)}`);
        console.warn('[desktop] web build failed, using the previous build:\n' + r.out.slice(-2000));
      }
    }
    status('Starting the cell server…');
    const port = await startServer();
    status('Loading the workspace…');
    openMain(port);
  } catch (err) {
    if (splash && !splash.isDestroyed()) splash.hide();
    dialog.showErrorBox('Pick Cell could not start', String(err && err.message ? err.message : err));
    stopping = true;
    await stopServer();
    app.exit(1);
  }
}

if (!app.requestSingleInstanceLock()) {
  app.quit(); // already open: the running window comes to the front instead
} else {
  app.on('second-instance', () => {
    const w = mainWindow || splash;
    if (w) { if (w.isMinimized()) w.restore(); w.show(); w.focus(); }
  });
  app.whenReady().then(() => {
    if (fs.existsSync(ICON)) app.dock?.setIcon(nativeImage.createFromPath(ICON));
    boot();
  });
  app.on('window-all-closed', () => quitApp());
  app.on('before-quit', (e) => { if (!stopping) { e.preventDefault(); quitApp(); } });
  for (const sig of ['SIGINT', 'SIGTERM']) process.on(sig, () => quitApp());
}
