/**
 * MuJoCo bridge (dev server only).
 *
 * POST /api/mujoco/simulate  { grasp, robotId, partFileName, partFileBase64, partWorldPose }
 *   -> writes jenga_mujoco/runs/<runId>/{part.stp, request.json}
 *   -> spawns `python jenga_mujoco/cad_pick_place.py --request ... --status ...`
 *      which opens a MuJoCo viewer window on this machine's display.
 * GET  /api/mujoco/status?id=<runId>
 *   -> contents of status.json written by the Python process
 *      ({ state: 'running' | 'done' | 'failed', phase, message, ok }).
 *
 * GET  /api/mujoco/file?id=<runId>&name=<file.jpg|png>
 *   -> an image written by the simulator into the run folder: feed.jpg (live inspection camera,
 *      refreshed ~7x per second) and inspect_XX.png (inspection stills).
 *
 * Automatic pick & place (jenga_mujoco/auto_pick_place.py):
 * POST /api/mujoco/auto/plan  { partFileName, partFileBase64, partWorldPose, robotId, gripperId, tcpOffset,
 *                               preGraspDistance, points: [{ point, normal }, { point, normal }] }
 *   -> plans in the background (no window); poll /api/mujoco/status with the returned runId.
 * GET  /api/mujoco/auto/plan?id=<runId>  -> plan.json: candidate grasps with their complete motions.
 * POST /api/mujoco/auto/run   { runId, candidate, mode: 'preview' | 'execute' }
 *   -> opens the MuJoCo window for that plan; poll /api/mujoco/status again.
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

function isFiniteVec(v: any): boolean {
  return !!v && [v.x, v.y, v.z].every((c) => typeof c === 'number' && Number.isFinite(c));
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
  const autoScript = path.join(simDir, 'auto_pick_place.py');
  const python =
    process.env.MUJOCO_PYTHON || path.join(os.homedir(), 'miniforge3/envs/mujoco_cad/bin/python');

  let child: ChildProcess | null = null;
  let childRunId: string | null = null;

  const pythonMissing = () =>
    `Python not found at ${python}. Create the env (see jenga_mujoco/CAD_SIM.md) or set MUJOCO_PYTHON.`;

  const newRunDir = (partFileName: string, partFileBase64: string) => {
    const runId = `${new Date().toISOString().replace(/[:.]/g, '-')}`;
    const runDir = path.join(runsDir, runId);
    fs.mkdirSync(runDir, { recursive: true });
    const ext = path.extname(partFileName).toLowerCase() || '.stp';
    const partPath = path.join(runDir, `part${ext}`);
    fs.writeFileSync(partPath, Buffer.from(partFileBase64, 'base64'));
    return { runId, runDir, partPath };
  };

  /** Starts a simulator process for a run. Only one at a time: any previous one is stopped. */
  const launch = (runId: string, runDir: string, scriptPath: string, args: string[]) => {
    if (child && child.exitCode === null) child.kill('SIGTERM');

    const statusPath = path.join(runDir, 'status.json');
    fs.writeFileSync(statusPath, JSON.stringify({ state: 'running', phase: 'starting', message: 'Starting simulator...' }));

    const log = fs.createWriteStream(path.join(runDir, 'log.txt'), { flags: 'a' });
    const proc = spawn(python, [scriptPath, ...args, '--status', statusPath], {
      cwd: simDir,
      env: { ...process.env, DISPLAY: process.env.DISPLAY || ':0', PYTHONUNBUFFERED: '1' }
    });
    child = proc;
    childRunId = runId;
    const tee = (chunk: Buffer) => {
      log.write(chunk);
      process.stdout.write(chunk.toString().replace(/^/gm, '[mujoco] '));
    };
    proc.stdout?.on('data', tee);
    proc.stderr?.on('data', tee);
    proc.on('exit', (code) => {
      log.end();
      // A process replaced by a newer one must not overwrite the status of the new run.
      if (child !== proc) return;
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
  };

  return {
    name: 'mujoco-bridge',
    configureServer(server) {
      server.middlewares.use(async (req, res, next) => {
        const url = new URL(req.url || '/', 'http://localhost');

        if (url.pathname === '/api/mujoco/simulate' && req.method === 'POST') {
          try {
            if (!fs.existsSync(python)) return sendJson(res, 500, { ok: false, message: pythonMissing() });
            const body = JSON.parse(await readBody(req));
            if (!body?.grasp?.tcpPose || !body?.partFileBase64 || !body?.partFileName) {
              return sendJson(res, 400, { ok: false, message: 'Missing grasp or part file in request.' });
            }

            const robotId = body.robotId ?? 'cs66';
            if (typeof robotId !== 'string' || !/^[a-z0-9_]+$/.test(robotId)) {
              return sendJson(res, 400, { ok: false, message: 'Invalid robotId.' });
            }

            const { runId, runDir, partPath } = newRunDir(String(body.partFileName), body.partFileBase64);
            const requestPath = path.join(runDir, 'request.json');
            fs.writeFileSync(
              requestPath,
              JSON.stringify(
                {
                  partFile: partPath,
                  partFileName: body.partFileName,
                  partWorldPose: body.partWorldPose ?? null,
                  robotId,
                  grasp: body.grasp
                },
                null,
                2
              )
            );
            launch(runId, runDir, script, ['--request', requestPath]);
            return sendJson(res, 200, { ok: true, runId });
          } catch (err: any) {
            return sendJson(res, 500, { ok: false, message: err?.message || String(err) });
          }
        }

        if (url.pathname === '/api/mujoco/auto/plan' && req.method === 'POST') {
          try {
            if (!fs.existsSync(python)) return sendJson(res, 500, { ok: false, message: pythonMissing() });
            const body = JSON.parse(await readBody(req));
            if (!body?.partFileBase64 || !body?.partFileName) {
              return sendJson(res, 400, { ok: false, message: 'Missing part file in request.' });
            }
            const points = body?.points;
            if (!Array.isArray(points) || points.length !== 2 || !points.every((m) => isFiniteVec(m?.point) && isFiniteVec(m?.normal))) {
              return sendJson(res, 400, { ok: false, message: 'Mark two points on the part.' });
            }
            for (const id of [body.robotId, body.gripperId]) {
              if (typeof id !== 'string' || !/^[a-z0-9_]+$/.test(id)) {
                return sendJson(res, 400, { ok: false, message: 'Invalid robotId or gripperId.' });
              }
            }
            const { runId, runDir, partPath } = newRunDir(String(body.partFileName), body.partFileBase64);
            const requestPath = path.join(runDir, 'request.json');
            fs.writeFileSync(
              requestPath,
              JSON.stringify(
                {
                  partFile: partPath,
                  partFileName: body.partFileName,
                  partWorldPose: body.partWorldPose ?? null,
                  robotId: body.robotId,
                  gripperId: body.gripperId,
                  tcpOffset: body.tcpOffset,
                  preGraspDistance: body.preGraspDistance,
                  points
                },
                null,
                2
              )
            );
            launch(runId, runDir, autoScript, [
              '--request', requestPath, '--plan', path.join(runDir, 'plan.json'), '--mode', 'plan'
            ]);
            return sendJson(res, 200, { ok: true, runId });
          } catch (err: any) {
            return sendJson(res, 500, { ok: false, message: err?.message || String(err) });
          }
        }

        if (url.pathname === '/api/mujoco/auto/plan' && req.method === 'GET') {
          const id = url.searchParams.get('id') || '';
          if (!/^[\w-]+$/.test(id)) return sendJson(res, 400, { ok: false, message: 'Bad run id' });
          try {
            const plan = fs.readFileSync(path.join(runsDir, id, 'plan.json'), 'utf8');
            res.writeHead(200, { 'Content-Type': 'application/json' });
            return res.end(plan);
          } catch {
            return sendJson(res, 404, { ok: false, message: `No plan for run ${id}` });
          }
        }

        if (url.pathname === '/api/mujoco/auto/run' && req.method === 'POST') {
          try {
            if (!fs.existsSync(python)) return sendJson(res, 500, { ok: false, message: pythonMissing() });
            const body = JSON.parse(await readBody(req));
            const id = String(body?.runId ?? '');
            const candidate = Number(body?.candidate ?? 0);
            if (!/^[\w-]+$/.test(id) || !Number.isInteger(candidate) || candidate < 0) {
              return sendJson(res, 400, { ok: false, message: 'Bad run id or candidate.' });
            }
            if (body?.mode !== 'preview' && body?.mode !== 'execute') {
              return sendJson(res, 400, { ok: false, message: "mode must be 'preview' or 'execute'." });
            }
            const runDir = path.join(runsDir, id);
            const planPath = path.join(runDir, 'plan.json');
            if (!fs.existsSync(planPath)) return sendJson(res, 404, { ok: false, message: `No plan for run ${id}` });
            launch(id, runDir, autoScript, [
              '--request', path.join(runDir, 'request.json'), '--plan', planPath,
              '--mode', body.mode, '--candidate', String(candidate)
            ]);
            return sendJson(res, 200, { ok: true, runId: id });
          } catch (err: any) {
            return sendJson(res, 500, { ok: false, message: err?.message || String(err) });
          }
        }

        if (url.pathname === '/api/mujoco/file' && req.method === 'GET') {
          const id = url.searchParams.get('id') || '';
          const name = url.searchParams.get('name') || '';
          if (!/^[\w-]+$/.test(id) || !/^[\w-]+\.(jpg|png)$/.test(name)) {
            res.writeHead(400);
            return res.end('Bad request');
          }
          try {
            const data = fs.readFileSync(path.join(runsDir, id, name));
            res.writeHead(200, {
              'Content-Type': name.endsWith('.png') ? 'image/png' : 'image/jpeg',
              'Cache-Control': 'no-store'
            });
            return res.end(data);
          } catch {
            res.writeHead(404);
            return res.end('Not found');
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
