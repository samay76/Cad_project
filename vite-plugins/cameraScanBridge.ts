/**
 * Camera scan bridge (dev server only).
 *
 * Starts camera_scan_server.py (RealSense -> live point cloud -> CAD model) and forwards requests to it,
 * so the browser only ever talks to the Vite server:
 *
 * POST /api/scan/start      -> spawns the Python scan server (no-op when it already runs)
 * POST /api/scan/stop       -> stops it (releases the camera)
 * GET  /api/scan/proc       -> { running, log } : process state and the last log lines
 * ANY  /api/scan/cam/<path> -> forwarded to http://127.0.0.1:$SCAN_PORT/<path>
 *                              (status, stream/depth|cloud|points MJPEG, build, refit, depth, model.stl)
 *
 * Python interpreter: $CAMERA_PYTHON, default ~/miniforge3/envs/realsense_cam/bin/python
 * (needs pyrealsense2, open3d, opencv-python, numpy).
 */
import type { Plugin } from 'vite';
import type { ServerResponse } from 'http';
import http from 'http';
import { spawn, type ChildProcess } from 'child_process';
import os from 'os';
import fs from 'fs';
import path from 'path';

const LOG_LINES = 40;

function sendJson(res: ServerResponse, code: number, body: unknown) {
  res.writeHead(code, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' });
  res.end(JSON.stringify(body));
}

export function cameraScanBridgePlugin(): Plugin {
  const root = process.cwd();
  const script = path.join(root, 'camera_scan_server.py');
  const port = Number(process.env.SCAN_PORT || 8765);
  const python =
    process.env.CAMERA_PYTHON || path.join(os.homedir(), 'miniforge3/envs/realsense_cam/bin/python');

  let child: ChildProcess | null = null;
  let log: string[] = [];

  const running = () => !!child && child.exitCode === null && child.signalCode === null;

  const start = () => {
    if (running()) return;
    log = [];
    const proc = spawn(python, [script, '--port', String(port)], {
      cwd: root,
      env: { ...process.env, PYTHONUNBUFFERED: '1' }
    });
    child = proc;
    const tee = (chunk: Buffer) => {
      const text = chunk.toString();
      process.stdout.write(text.replace(/^/gm, '[scan] '));
      log.push(...text.split('\n').filter((l) => l.trim()));
      if (log.length > LOG_LINES) log = log.slice(-LOG_LINES);
    };
    proc.stdout?.on('data', tee);
    proc.stderr?.on('data', tee);
    proc.on('error', (err) => log.push(`Failed to start: ${err.message}`));
    proc.on('exit', (code, signal) => {
      log.push(`Scan server exited (${signal ?? `code ${code}`}).`);
    });
  };

  const stop = () => {
    if (running()) child!.kill('SIGTERM');
  };

  return {
    name: 'camera-scan-bridge',
    configureServer(server) {
      server.httpServer?.on('close', stop);
      process.once('exit', stop);

      server.middlewares.use((req, res, next) => {
        const url = new URL(req.url || '/', 'http://localhost');
        if (!url.pathname.startsWith('/api/scan/')) return next();

        if (url.pathname === '/api/scan/start' && req.method === 'POST') {
          if (!fs.existsSync(python)) {
            return sendJson(res, 500, {
              ok: false,
              message: `Python not found at ${python}. Create the env (see CAMERA_SCAN.md) or set CAMERA_PYTHON.`
            });
          }
          start();
          return sendJson(res, 200, { ok: true });
        }
        if (url.pathname === '/api/scan/stop' && req.method === 'POST') {
          stop();
          return sendJson(res, 200, { ok: true });
        }
        if (url.pathname === '/api/scan/proc' && req.method === 'GET') {
          return sendJson(res, 200, { running: running(), log: log.slice(-12) });
        }

        if (url.pathname.startsWith('/api/scan/cam/')) {
          const target = url.pathname.slice('/api/scan/cam'.length) + url.search;
          const upstream = http.request(
            { host: '127.0.0.1', port, path: target, method: req.method, headers: { ...req.headers, host: `127.0.0.1:${port}` } },
            (up) => {
              res.writeHead(up.statusCode || 502, up.headers);
              up.pipe(res);
            }
          );
          upstream.on('error', () => {
            if (res.headersSent) return res.end();
            sendJson(res, 503, {
              ok: false,
              running: running(),
              message: running() ? 'The camera server is starting...' : 'The camera server is not running.'
            });
          });
          // A closed browser tab / <img> must end the MJPEG stream upstream too.
          res.on('close', () => upstream.destroy());
          req.pipe(upstream);
          return;
        }

        return sendJson(res, 404, { ok: false, message: 'Unknown scan endpoint' });
      });
    }
  };
}
