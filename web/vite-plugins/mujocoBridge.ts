/**
 * MuJoCo bridge (dev server only).
 *
 * GET  /api/mujoco/status?id=<runId>
 *   -> contents of status.json written by the Python process
 *      ({ state: 'running' | 'done' | 'failed', phase, message, ok }).
 *
 * Automatic pick & place (cell/auto_pick_place.py):
 * POST /api/mujoco/auto/plan  { partFileName, partFileBase64, partWorldPose, robotId, gripperId, tcpOffset,
 *                               preGraspDistance, points: [{ point, normal }, { point, normal }] }
 *   -> plans in the background (no window); poll /api/mujoco/status with the returned runId.
 * GET  /api/mujoco/auto/plan?id=<runId>  -> plan.json: candidate grasps with their complete motions.
 * POST /api/mujoco/auto/run   { runId, candidate, mode: 'preview' | 'execute' }
 *   -> opens the MuJoCo window for that plan; poll /api/mujoco/status again.
 *
 * POST /api/mujoco/auto/real  { runId, candidate, robotIp, robotSrc?, speed, gripper?: boolean, action?: 'real' | 'home' }
 *   gripper: drive the real Robotiq 2F-85 on USB during Real Sim (config/cell.json real_gripper)
 *   ("Real Sim", or "Go Home": only bring the robot to the plan's home pose)
 *   -> moves the real Elite EC66 through the plan's joints (auto_pick_place.py --mode real, real_robot.py),
 *      the MuJoCo window following it live. robotIp 'fake' = simulated robot.
 * POST /api/mujoco/stop  -> SIGTERM to the running simulator; in real mode that sends `stop` to the robot.
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
  const repo = path.resolve(process.cwd(), '..'); // the dev server runs in web/
  const runsDir = path.join(process.env.CELL_DATA_DIR || path.join(repo, 'var'), 'runs');
  const configFile = process.env.CELL_CONFIG || path.join(repo, 'config', 'cell.json');
  const python =
    process.env.MUJOCO_PYTHON || path.join(os.homedir(), 'miniforge3/envs/mujoco_cad/bin/python');

  let child: ChildProcess | null = null;
  let childRunId: string | null = null;

  const pythonMissing = () =>
    `Python not found at ${python}. Create the env (see README.md) or set MUJOCO_PYTHON.`;

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
  const launch = (runId: string, runDir: string, args: string[]) => {
    if (child && child.exitCode === null) child.kill('SIGTERM');

    const statusPath = path.join(runDir, 'status.json');
    fs.writeFileSync(statusPath, JSON.stringify({ state: 'running', phase: 'starting', message: 'Starting simulator...' }));

    const log = fs.createWriteStream(path.join(runDir, 'log.txt'), { flags: 'a' });
    const proc = spawn(python, ['-m', 'cell.auto_pick_place', ...args, '--status', statusPath], {
      cwd: repo,
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
                  points,
                  pickOffsetMm: isFiniteVec(body?.pickOffsetMm) ? body.pickOffsetMm : { x: 0, y: 0, z: 0 },
                  // gripper pose set in the 3D view (Step Jogging): planned exactly, part frame mm, quaternion xyzw
                  graspPose:
                    isFiniteVec(body?.graspPose?.position) &&
                    ['x', 'y', 'z', 'w'].every((k) => Number.isFinite(body?.graspPose?.orientation?.[k]))
                      ? body.graspPose
                      : null
                },
                null,
                2
              )
            );
            launch(runId, runDir, [
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
            launch(id, runDir, [
              '--request', path.join(runDir, 'request.json'), '--plan', planPath,
              '--mode', body.mode, '--candidate', String(candidate)
            ]);
            return sendJson(res, 200, { ok: true, runId: id });
          } catch (err: any) {
            return sendJson(res, 500, { ok: false, message: err?.message || String(err) });
          }
        }

        if (url.pathname === '/api/mujoco/auto/real' && req.method === 'POST') {
          try {
            if (!fs.existsSync(python)) return sendJson(res, 500, { ok: false, message: pythonMissing() });
            const body = JSON.parse(await readBody(req));
            const id = String(body?.runId ?? '');
            const candidate = Number(body?.candidate ?? 0);
            const ip = String(body?.robotIp ?? '').trim();
            const src = String(body?.robotSrc ?? '').trim();
            const speed = Number(body?.speed ?? 5);
            const action = body?.action === 'home' ? 'home' : 'real'; // 'home' = Go Home only
            const ipv4 = /^\d{1,3}(\.\d{1,3}){3}$/;
            if (!/^[\w-]+$/.test(id) || !Number.isInteger(candidate) || candidate < 0) {
              return sendJson(res, 400, { ok: false, message: 'Bad run id or candidate.' });
            }
            if (ip !== 'fake' && !ipv4.test(ip)) {
              return sendJson(res, 400, { ok: false, message: "Robot IP must look like 192.168.1.200 (or 'fake')." });
            }
            if (src && !ipv4.test(src)) return sendJson(res, 400, { ok: false, message: 'Bad source IP.' });
            if (!Number.isFinite(speed) || speed < 1 || speed > 10) {
              return sendJson(res, 400, { ok: false, message: 'Speed must be 1..10 %.' });
            }
            const runDir = path.join(runsDir, id);
            const planPath = path.join(runDir, 'plan.json');
            if (!fs.existsSync(planPath)) return sendJson(res, 404, { ok: false, message: `No plan for run ${id}` });
            launch(id, runDir, [
              '--request', path.join(runDir, 'request.json'), '--plan', planPath,
              '--mode', action, '--candidate', String(candidate),
              '--robot-ip', ip, '--speed', String(speed), ...(src ? ['--robot-src', src] : []),
              ...(body?.gripper === true && action === 'real' ? ['--real-gripper'] : [])
            ]);
            return sendJson(res, 200, { ok: true, runId: id });
          } catch (err: any) {
            return sendJson(res, 500, { ok: false, message: err?.message || String(err) });
          }
        }

        // Stops the running simulator process. In real mode its SIGTERM handler sends `stop` to the robot.
        if (url.pathname === '/api/mujoco/stop' && req.method === 'POST') {
          if (child && child.exitCode === null) {
            child.kill('SIGTERM');
            return sendJson(res, 200, { ok: true, runId: childRunId });
          }
          return sendJson(res, 200, { ok: true, runId: null, message: 'Nothing running.' });
        }

        if (url.pathname === '/api/mujoco/fixture' && req.method === 'GET') {
          // The pick fixture and the part made for it (config/cell.json pick_station), for the 3D view.
          try {
            const cfg = JSON.parse(fs.readFileSync(configFile, 'utf8'));
            const ps = cfg?.pick_station ?? {};
            const seatAll = !!ps.seat_all_parts;
            if (!ps.enabled || !ps.file || (!ps.seated_part && !seatAll)) return sendJson(res, 200, { ok: true, fixture: null });
            const abs = (p: string) => (path.isAbsolute(p) ? p : path.join(repo, p));
            const fixture = fs.readFileSync(abs(ps.file));
            const seated = ps.seated_part ? fs.readFileSync(abs(ps.seated_part)) : Buffer.alloc(0);
            const { createHash } = await import('crypto');
            return sendJson(res, 200, {
              ok: true,
              fixture: {
                name: path.basename(ps.file),
                base64: fixture.toString('base64'),
                seatedSha256: createHash('sha256').update(seated).digest('hex'),
                seatAll
              }
            });
          } catch (err: any) {
            return sendJson(res, 200, { ok: true, fixture: null, message: err?.message || String(err) });
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
