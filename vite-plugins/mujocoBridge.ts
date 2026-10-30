/**
 * MuJoCo bridge (dev server only).
 *
 * POST /api/mujoco/simulate  { grasp, partFileName, partFileBase64, partWorldPose }
 *   -> writes jenga_mujoco/runs/<runId>/{part.stp, request.json}
 *   -> spawns `python jenga_mujoco/cad_pick_place.py --request ... --status ...`
 *      which opens a MuJoCo viewer window on this machine's display.
 * GET  /api/mujoco/status?id=<runId>
 *   -> contents of status.json written by the Python process
 *      ({ state: 'running' | 'done' | 'failed', phase, message, ok }).
 *
 * Python interpreter: $MUJOCO_PYTHON, default ~/miniforge3/envs/mujoco_cad/bin/python.
 */
import type { Plugin } from 'vite';
import type { IncomingMessage, ServerResponse } from 'http';
import { spawn, type ChildProcess } from 'child_process';
import fs from 'fs';
import os from 'os';
import path from 'path';

const MAX_BODY_BYTES = 64 * 1024 * 1024;

function sendJson(res: ServerResponse, code: number, body: unknown) {
  res.writeHead(code, { 'Content-Type': 'application/json' });
  res.end(JSON.stringify(body));
}

function readBody(req: IncomingMessage): Promise<string> {
  return new Promise((resolve, reject) => {
    let size = 0;
    const chunks: Buffer[] = [];
    req.on('data', (c: Buffer) => {
      size += c.length;
      if (size > MAX_BODY_BYTES) {
        reject(new Error('Request too large'));
        req.destroy();
        return;
      }
      chunks.push(c);
    });
    req.on('end', () => resolve(Buffer.concat(chunks).toString('utf8')));
    req.on('error', reject);
  });
}

export function mujocoBridgePlugin(): Plugin {
  const simDir = path.resolve(process.cwd(), 'jenga_mujoco');
  const runsDir = path.join(simDir, 'runs');
  const script = path.join(simDir, 'cad_pick_place.py');
  const python =
    process.env.MUJOCO_PYTHON || path.join(os.homedir(), 'miniforge3/envs/mujoco_cad/bin/python');

  let child: ChildProcess | null = null;
  let childRunId: string | null = null;

  return {
    name: 'mujoco-bridge',
    configureServer(server) {
      server.middlewares.use(async (req, res, next) => {
        const url = new URL(req.url || '/', 'http://localhost');

        if (url.pathname === '/api/mujoco/simulate' && req.method === 'POST') {
          try {
            if (!fs.existsSync(python)) {
              return sendJson(res, 500, {
                ok: false,
                message: `Python not found at ${python}. Create the env (see jenga_mujoco/CAD_SIM.md) or set MUJOCO_PYTHON.`
              });
            }
            const body = JSON.parse(await readBody(req));
            if (!body?.grasp?.tcpPose || !body?.partFileBase64 || !body?.partFileName) {
              return sendJson(res, 400, { ok: false, message: 'Missing grasp or part file in request.' });
            }

            // Only one simulator window at a time.
            if (child && child.exitCode === null) child.kill('SIGTERM');

            const runId = `${new Date().toISOString().replace(/[:.]/g, '-')}`;
            const runDir = path.join(runsDir, runId);
            fs.mkdirSync(runDir, { recursive: true });

            const ext = path.extname(String(body.partFileName)).toLowerCase() || '.stp';
            const partPath = path.join(runDir, `part${ext}`);
            fs.writeFileSync(partPath, Buffer.from(body.partFileBase64, 'base64'));

            const requestPath = path.join(runDir, 'request.json');
            fs.writeFileSync(
              requestPath,
              JSON.stringify(
                {
                  partFile: partPath,
                  partFileName: body.partFileName,
                  partWorldPose: body.partWorldPose ?? null,
                  grasp: body.grasp
                },
                null,
                2
              )
            );
            const statusPath = path.join(runDir, 'status.json');
            fs.writeFileSync(statusPath, JSON.stringify({ state: 'running', phase: 'starting', message: 'Starting simulator...' }));

            const log = fs.createWriteStream(path.join(runDir, 'log.txt'));
            child = spawn(python, [script, '--request', requestPath, '--status', statusPath], {
              cwd: simDir,
              env: { ...process.env, DISPLAY: process.env.DISPLAY || ':0', PYTHONUNBUFFERED: '1' }
            });
            childRunId = runId;
            const tee = (chunk: Buffer) => {
              log.write(chunk);
              process.stdout.write(chunk.toString().replace(/^/gm, '[mujoco] '));
            };
            child.stdout?.on('data', tee);
            child.stderr?.on('data', tee);
            child.on('exit', (code) => {
              log.end();
              // If Python died without reporting a final state, surface the log tail.
              try {
                const st = JSON.parse(fs.readFileSync(statusPath, 'utf8'));
                if (st.state === 'running') {
                  const tail = fs.readFileSync(path.join(runDir, 'log.txt'), 'utf8').trim().split('\n').slice(-6).join('\n');
                  fs.writeFileSync(
                    statusPath,
                    JSON.stringify({ state: 'failed', ok: false, phase: 'crashed', message: `Simulator exited (code ${code}).\n${tail}` })
                  );
                }
              } catch {
                // ignore
              }
            });

            return sendJson(res, 200, { ok: true, runId });
          } catch (err: any) {
            return sendJson(res, 500, { ok: false, message: err?.message || String(err) });
          }
        }

        if (url.pathname === '/api/mujoco/status' && req.method === 'GET') {
          const id = url.searchParams.get('id') || childRunId || '';
          if (!/^[\w-]+$/.test(id)) return sendJson(res, 400, { state: 'failed', ok: false, message: 'Bad run id' });
          try {
            const st = fs.readFileSync(path.join(runsDir, id, 'status.json'), 'utf8');
            res.writeHead(200, { 'Content-Type': 'application/json' });
            return res.end(st);
          } catch {
            return sendJson(res, 404, { state: 'failed', ok: false, message: `Unknown run ${id}` });
          }
        }

        next();
      });
    }
  };
}
