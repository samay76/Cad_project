"""
jobs.py - Runs the cell's Python jobs (plan, preview, sim, real robot) as child processes, one at a time.

Each web request gets a run folder var/runs/<run id>/ with the part file, request.json, plan.json, status.json (the
job's progress, polled by the web app) and log.txt. Starting a job stops the one before it. Stopping sends SIGTERM:
in real mode the job's handler then sends `stop` to the robot.
"""
import json
import os
import re
import shutil
import signal
import subprocess
import threading
from datetime import datetime, timezone

RUN_ID = re.compile(r'^[\w-]{1,64}$')
_PART_EXT = {'.step', '.stp', '.stl', '.obj'}


class JobRunner:
    def __init__(self, settings, repo, module='cell.auto_pick_place'):
        self.s, self.repo, self.module = settings, repo, module
        self._lock = threading.Lock()
        self._proc = None
        self._run_id = None
        os.makedirs(settings.runs_dir, exist_ok=True)

    # ---------------- run folders ----------------
    def run_dir(self, run_id):
        if not RUN_ID.match(run_id or ''):
            raise ValueError('Bad run id.')
        return os.path.join(self.s.runs_dir, run_id)

    def new_run(self, part_file_name, part_bytes):
        ext = os.path.splitext(part_file_name)[1].lower() or '.stp'
        if ext not in _PART_EXT:
            raise ValueError(f'Unsupported part file type {ext!r} (STEP, STL or OBJ).')
        run_id = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H-%M-%S-%f')[:-3] + 'Z'
        run_dir = self.run_dir(run_id)
        os.makedirs(run_dir, exist_ok=False)
        part_path = os.path.join(run_dir, f'part{ext}')
        with open(part_path, 'wb') as f:
            f.write(part_bytes)
        self._prune()
        return run_id, run_dir, part_path

    def _prune(self):
        """Keeps the newest keep_runs run folders (the running one is never deleted)."""
        runs = sorted(d for d in os.listdir(self.s.runs_dir) if RUN_ID.match(d))
        for old in runs[:max(0, len(runs) - self.s.keep_runs)]:
            if old != self._run_id:
                shutil.rmtree(os.path.join(self.s.runs_dir, old), ignore_errors=True)

    # ---------------- processes ----------------
    @property
    def running(self):
        return self._proc is not None and self._proc.poll() is None

    @property
    def current_run(self):
        return self._run_id

    def launch(self, run_id, args):
        """Starts `python -m <module> <args> --status <run>/status.json`; stops a running job first."""
        run_dir = self.run_dir(run_id)
        status_path = os.path.join(run_dir, 'status.json')
        with self._lock:
            self._stop_locked(wait=True)
            _write_json(status_path, {'state': 'running', 'phase': 'starting', 'message': 'Starting...'})
            log = open(os.path.join(run_dir, 'log.txt'), 'ab')
            # PYTHONPATH = the repo only: a ROS (or other) PYTHONPATH from the shell must not leak into the job
            env = dict(os.environ, DISPLAY=self.s.display, PYTHONUNBUFFERED='1', PYTHONPATH=self.repo,
                       CELL_CONFIG=self.s.config_file, CELL_DATA_DIR=self.s.data_dir)
            proc = subprocess.Popen([self.s.python, '-m', self.module, *args, '--status', status_path],
                                    cwd=self.repo, env=env, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            self._proc, self._run_id = proc, run_id
        threading.Thread(target=self._watch, args=(proc, log, run_dir, status_path), daemon=True).start()

    def _watch(self, proc, log, run_dir, status_path):
        code = proc.wait()
        log.close()
        if self._proc is not proc:   # replaced by a newer job: leave its status alone
            return
        try:
            st = _read_json(status_path)
        except (OSError, ValueError):
            st = {'state': 'running'}
        if st.get('state') == 'running':   # died without reporting: show the end of its log
            tail = _tail(os.path.join(run_dir, 'log.txt'), 6)
            _write_json(status_path, {'state': 'failed', 'ok': False, 'phase': 'crashed',
                                      'message': f'The job exited (code {code}).\n{tail}'})

    def stop(self):
        with self._lock:
            run_id = self._run_id if self.running else None
            self._stop_locked(wait=False)
            return run_id

    def _stop_locked(self, wait):
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        proc.send_signal(signal.SIGTERM)   # real mode: sends `stop` to the robot, then exits
        if wait:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    def shutdown(self):
        with self._lock:
            self._stop_locked(wait=True)


def _write_json(path, body):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(body, f)
    os.replace(tmp, path)


def _read_json(path):
    with open(path) as f:
        return json.load(f)


def _tail(path, n):
    try:
        with open(path, 'rb') as f:
            return b'\n'.join(f.read().strip().split(b'\n')[-n:]).decode('utf8', 'replace')
    except OSError:
        return ''
