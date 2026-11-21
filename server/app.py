"""
app.py - HTTP API of the pick cell and the built web app, in one server.

The API is the one the web app calls (web/src/autopick/api.ts); each job runs as a child process (jobs.py):

  POST /api/mujoco/auto/plan   part file + marked points (grip: 2, touch: 1) -> plans in the background; { runId }
  GET  /api/mujoco/auto/plan?id=<runId>   plan.json of that run
  POST /api/mujoco/auto/run    { runId, candidate, mode: 'preview' | 'execute' }   MuJoCo window on this PC
  POST /api/mujoco/auto/real   { runId, candidate, robotIp, robotSrc?, speed, gripper?, action?: 'real' | 'home' }
                               moves the real EC66 ('fake' = simulated robot), mirrored in MuJoCo
  POST /api/mujoco/stop        stops the running job (real mode: sends `stop` to the robot)
  GET  /api/mujoco/status?id=<runId>      the job's progress (status.json)
  GET  /api/mujoco/fixture     the pick fixture STEP for the 3D view
  GET  /api/mujoco/view/scene?id=<runId>          the running job's cell (web_view.py), 404 until it is ready
  GET  /api/mujoco/view/frames?id=<runId>&from=N  live body poses from frame N on (float32 frames)
  GET  /api/mujoco/view/mesh/<hash>               one mesh of the cell (cached by the browser)
  GET  /api/cell/scene, /api/cell/frames           the cell without a part (EC66 at home), for the teaching view
  GET  /api/health             server, config and job state
  /api/library/...             fixture library: fixtures and their STEP parts (library.py)

Everything else serves web/dist (npm run build). Nothing outside web/dist and the run folders is served.
"""
import base64
import binascii
import hashlib
import ipaddress
import json
import os
import re
from contextlib import asynccontextmanager
from typing import List, Literal, Optional

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, ValidationError, field_validator, model_validator

from .jobs import JobRunner
from .settings import REPO, load_settings

MAX_BODY_BYTES = 64 * 1024 * 1024
IDENT = r'^[a-z0-9_]{1,32}$'
MESH_KEY = re.compile(r'^[0-9a-f]{32}$')


class Vec3(BaseModel):
    x: float
    y: float
    z: float


class Quat(BaseModel):
    x: float
    y: float
    z: float
    w: float


class Pose(BaseModel):
    position: Vec3
    orientation: Quat


class MarkedPoint(BaseModel):
    point: Vec3
    normal: Vec3


class PlanBody(BaseModel):
    partFileName: str = Field(min_length=1, max_length=200)
    partFileBase64: str = Field(min_length=1)
    partWorldPose: Optional[Pose] = None
    robotId: str = Field(pattern=IDENT)
    gripperId: str = Field(pattern=IDENT)
    tcpOffset: Optional[float] = None
    preGraspDistance: Optional[float] = None
    points: List[MarkedPoint] = Field(min_length=1, max_length=2)
    mode: Literal['grip', 'touch'] = 'grip'   # touch: the closed fingertips go to the one marked point
    pickOffsetMm: Optional[Vec3] = None
    graspPose: Optional[Pose] = None
    fixtureId: Optional[str] = Field(None, pattern=r'^[0-9a-f]{12}$')   # fixture library: plan on this fixture

    @model_validator(mode='after')
    def _count(self):
        if len(self.points) != (1 if self.mode == 'touch' else 2):
            raise ValueError('mark one point for touch, two points (one per finger) for grip')
        return self


class RunBody(BaseModel):
    runId: str
    candidate: int = Field(0, ge=0)
    mode: Literal['preview', 'approach', 'execute']


class RealBody(BaseModel):
    runId: str
    candidate: int = Field(0, ge=0)
    robotIp: str
    robotSrc: Optional[str] = None
    speed: float = Field(5.0, ge=1, le=10)
    gripper: bool = False
    action: Literal['real', 'home'] = 'real'

    @field_validator('robotIp')
    @classmethod
    def _ip(cls, v):
        v = v.strip()
        if v != 'fake':
            ipaddress.IPv4Address(v)
        return v

    @field_validator('robotSrc')
    @classmethod
    def _src(cls, v):
        v = (v or '').strip()
        if v:
            ipaddress.IPv4Address(v)
        return v or None


def _bad(message, code=400):
    return JSONResponse({'ok': False, 'message': message}, status_code=code)


def create_app(settings=None):
    s = settings or load_settings()
    from cell.config_check import check
    problems = check(s.config_file)
    if problems:
        raise SystemExit(f'{s.config_file} has problems (the server does not start):\n  ' + '\n  '.join(problems))
    jobs = JobRunner(s, REPO)

    @asynccontextmanager
    async def lifespan(_app):
        yield
        jobs.shutdown()   # stops a running job (real mode: the robot) when the server stops

    app = FastAPI(title='Pick cell', docs_url=None, redoc_url=None, lifespan=lifespan)
    app.state.jobs, app.state.settings = jobs, s

    async def body_of(request, model):
        size = int(request.headers.get('content-length') or 0)
        if size > MAX_BODY_BYTES:
            raise HTTPException(413, 'Request too large.')
        raw = await request.body()
        if len(raw) > MAX_BODY_BYTES:
            raise HTTPException(413, 'Request too large.')
        try:
            return model.model_validate(json.loads(raw))
        except (ValueError, ValidationError) as e:
            raise HTTPException(400, _short(e)) from None

    def plan_path(run_id):
        try:
            path = os.path.join(jobs.run_dir(run_id), 'plan.json')
        except ValueError as e:
            raise HTTPException(400, str(e)) from None
        if not os.path.exists(path):
            raise HTTPException(404, f'No plan for run {run_id}.')
        return path

    @app.exception_handler(HTTPException)
    async def _http_error(_request, exc):
        return _bad(exc.detail, exc.status_code)

    @app.get('/api/health')
    def health():
        return {'ok': True, 'running': jobs.running, 'runId': jobs.current_run, 'config': s.config_file}

    @app.post('/api/mujoco/auto/plan')
    async def plan(request: Request):
        b = await body_of(request, PlanBody)
        try:
            part = base64.b64decode(b.partFileBase64, validate=True)
        except (binascii.Error, ValueError):
            return _bad('The part file is not valid base64.')
        try:
            run_id, run_dir, part_path = jobs.new_run(b.partFileName, part)
        except ValueError as e:
            return _bad(str(e))
        fixture_file = None
        if b.fixtureId:
            fixture_file = app.state.library.fixture_path(b.fixtureId)
            if not fixture_file:
                return _bad('This fixture has no fixture CAD file. Add it in the fixture window first.')
        request_path = os.path.join(run_dir, 'request.json')
        with open(request_path, 'w') as f:
            json.dump({
                'partFile': part_path, 'partFileName': b.partFileName,
                'partWorldPose': b.partWorldPose.model_dump() if b.partWorldPose else None,
                'robotId': b.robotId, 'gripperId': b.gripperId, 'tcpOffset': b.tcpOffset,
                'preGraspDistance': b.preGraspDistance, 'points': [p.model_dump() for p in b.points], 'mode': b.mode,
                'pickOffsetMm': (b.pickOffsetMm or Vec3(x=0, y=0, z=0)).model_dump(),
                'graspPose': b.graspPose.model_dump() if b.graspPose else None,
                'fixtureFile': fixture_file,
            }, f, indent=2)
        jobs.launch(run_id, ['--request', request_path, '--plan', os.path.join(run_dir, 'plan.json'), '--mode', 'plan'])
        return {'ok': True, 'runId': run_id}

    @app.get('/api/mujoco/auto/plan')
    def get_plan(id: str = ''):
        return FileResponse(plan_path(id), media_type='application/json')

    @app.post('/api/mujoco/auto/run')
    async def run(request: Request):
        b = await body_of(request, RunBody)
        path = plan_path(b.runId)
        run_dir = os.path.dirname(path)
        jobs.launch(b.runId, ['--request', os.path.join(run_dir, 'request.json'), '--plan', path,
                              '--mode', b.mode, '--candidate', str(b.candidate)])
        return {'ok': True, 'runId': b.runId}

    @app.post('/api/mujoco/auto/real')
    async def real(request: Request):
        b = await body_of(request, RealBody)
        path = plan_path(b.runId)
        run_dir = os.path.dirname(path)
        args = ['--request', os.path.join(run_dir, 'request.json'), '--plan', path, '--mode', b.action,
                '--candidate', str(b.candidate), '--robot-ip', b.robotIp, '--speed', str(b.speed)]
        if b.robotSrc:
            args += ['--robot-src', b.robotSrc]
        if b.gripper and b.action == 'real':
            args.append('--real-gripper')
        jobs.launch(b.runId, args)
        return {'ok': True, 'runId': b.runId}

    @app.post('/api/mujoco/stop')
    def stop():
        run_id = jobs.stop()
        return {'ok': True, 'runId': run_id, **({} if run_id else {'message': 'Nothing running.'})}

    @app.get('/api/mujoco/status')
    def status(id: str = ''):
        run_id = id or jobs.current_run or ''
        try:
            path = os.path.join(jobs.run_dir(run_id), 'status.json')
        except ValueError:
            return JSONResponse({'state': 'failed', 'ok': False, 'message': 'Bad run id'}, status_code=400)
        if not os.path.exists(path):
            return JSONResponse({'state': 'failed', 'ok': False, 'message': f'Unknown run {run_id}'}, status_code=404)
        return FileResponse(path, media_type='application/json', headers={'Cache-Control': 'no-store'})

    @app.get('/api/mujoco/fixture')
    def fixture(library: str = ''):
        from cell.paths import load_config, repo_path
        if library:   # a fixture of the fixture library: every part goes on it
            path = app.state.library.fixture_path(library)
            if not path:
                return {'ok': True, 'fixture': None}
            with open(path, 'rb') as f:
                data = f.read()
            return {'ok': True, 'fixture': {'name': app.state.library.fixture_file_name(library),
                                            'base64': base64.b64encode(data).decode('ascii'),
                                            'seatedSha256': '', 'seatAll': True}}
        ps = load_config(s.config_file).get('pick_station') or {}
        seat_all = bool(ps.get('seat_all_parts'))
        if not ps.get('enabled') or not ps.get('file') or not (ps.get('seated_part') or seat_all):
            return {'ok': True, 'fixture': None}
        with open(repo_path(ps['file']), 'rb') as f:
            data = f.read()
        seated = b''
        if ps.get('seated_part'):
            with open(repo_path(ps['seated_part']), 'rb') as f:
                seated = f.read()
        return {'ok': True, 'fixture': {'name': os.path.basename(ps['file']),
                                        'base64': base64.b64encode(data).decode('ascii'),
                                        'seatedSha256': hashlib.sha256(seated).hexdigest(), 'seatAll': seat_all}}

    def run_file(run_id, name):
        try:
            return os.path.join(jobs.run_dir(run_id), name)
        except ValueError as e:
            raise HTTPException(400, str(e)) from None

    @app.get('/api/mujoco/view/scene')
    def view_scene(id: str = ''):
        path = run_file(id, 'scene.json')
        if not os.path.exists(path):
            return JSONResponse({'ok': False, 'message': 'No live view yet.'}, status_code=404,
                                headers={'Cache-Control': 'no-store'})
        return FileResponse(path, media_type='application/json', headers={'Cache-Control': 'no-store'})

    @app.get('/api/mujoco/view/frames')
    def view_frames(id: str = '', start: int = Query(0, alias='from', ge=0)):
        scene, frames = run_file(id, 'scene.json'), run_file(id, 'frames.bin')
        try:
            with open(scene) as f:
                frame_bytes = 4 * int(json.load(f)['frameFloats'])
            with open(frames, 'rb') as f:
                f.seek(start * frame_bytes)
                data = f.read()
        except (OSError, ValueError, KeyError):
            data, frame_bytes = b'', 1
        data = data[:len(data) - len(data) % frame_bytes]   # whole frames only
        return Response(data, media_type='application/octet-stream', headers={'Cache-Control': 'no-store'})

    @app.get('/api/mujoco/view/mesh/{key}')
    def view_mesh(key: str):
        if not MESH_KEY.match(key):
            raise HTTPException(400, 'Bad mesh id.')
        path = os.path.join(s.data_dir, 'cache', 'webview', f'{key}.bin')
        if not os.path.exists(path):
            raise HTTPException(404, 'Unknown mesh.')
        return FileResponse(path, media_type='application/octet-stream',
                            headers={'Cache-Control': 'public, max-age=31536000, immutable'})

    cell_dir = {'path': None}

    def cell_file(name):
        # python -m cell.cell_scene builds (or finds, cached per config) the empty cell and prints its folder; the
        # answer is kept while the config file is unchanged, so a page load does not start Python
        st = os.stat(s.config_file)
        key = (st.st_mtime_ns, st.st_size)
        if cell_dir['path'] and cell_dir.get('key') == key and os.path.exists(os.path.join(cell_dir['path'], name)):
            return os.path.join(cell_dir['path'], name)
        import subprocess
        env = dict(os.environ, PYTHONPATH=REPO, CELL_CONFIG=s.config_file, CELL_DATA_DIR=s.data_dir)
        r = subprocess.run([s.python, '-m', 'cell.cell_scene'], cwd=REPO, env=env, capture_output=True, text=True,
                           timeout=300)
        if r.returncode != 0:
            raise HTTPException(500, 'Could not build the cell: ' + (r.stderr.strip().splitlines() or ['?'])[-1])
        cell_dir['path'] = r.stdout.strip().splitlines()[-1]
        cell_dir['key'] = key
        return os.path.join(cell_dir['path'], name)

    def warm_cell():
        try:
            cell_file('scene.json')
        except Exception as e:  # noqa: BLE001 - the page builds it on demand then
            print(f'[server] could not prepare the workspace: {e}', flush=True)

    import threading
    threading.Thread(target=warm_cell, daemon=True).start()   # ready before the first page load

    @app.get('/api/cell/scene')
    def cell_scene():
        return FileResponse(cell_file('scene.json'), media_type='application/json', headers={'Cache-Control': 'no-store'})

    @app.get('/api/cell/frames')
    def cell_frames():
        path = os.path.join(cell_dir['path'], 'frames.bin') if cell_dir['path'] else cell_file('frames.bin')
        return FileResponse(path, media_type='application/octet-stream', headers={'Cache-Control': 'no-store'})

    from .library import register as register_library
    register_library(app, s.data_dir, body_of)   # fixture library: /api/library/...

    @app.api_route('/api/{rest:path}', methods=['GET', 'POST', 'PUT', 'PATCH', 'DELETE'])
    def api_unknown(rest: str):
        return _bad(f'Unknown API /api/{rest}', 404)

    if os.path.isdir(s.web_dist):
        app.mount('/', StaticFiles(directory=s.web_dist, html=True), name='web')
    else:
        @app.get('/')
        def no_web():
            return _bad('The web app is not built: run `npm run build` in web/.', 503)
    return app


def _short(e):
    if isinstance(e, ValidationError):
        err = e.errors()[0]
        return f"Invalid request: {'.'.join(str(p) for p in err['loc'])}: {err['msg']}"
    return f'Invalid request: {e}'
