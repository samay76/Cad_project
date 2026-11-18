"""The HTTP server: request validation, what it serves (and refuses to serve), one job at a time."""
import base64
import dataclasses
import json
import os
import sys
import time

import pytest
from fastapi.testclient import TestClient

from conftest import FIXTURES, REPO, write_config
from server.app import create_app
from server.jobs import JobRunner
from server.settings import load_settings


@pytest.fixture
def settings(tmp_path):
    return dataclasses.replace(load_settings(), data_dir=str(tmp_path / 'var'), python=sys.executable,
                               config_file=write_config(tmp_path))


@pytest.fixture
def client(settings):
    with TestClient(create_app(settings)) as c:
        yield c


def _plan_body(**over):
    with open(os.path.join(FIXTURES, 'main_part_request.json')) as f:
        req = json.load(f)
    with open(os.path.join(REPO, 'cad', 'main_part.step'), 'rb') as f:
        part = base64.b64encode(f.read()).decode()
    body = {k: req[k] for k in ('partWorldPose', 'robotId', 'gripperId', 'tcpOffset', 'preGraspDistance',
                                'points', 'pickOffsetMm')}
    return {**body, 'partFileName': 'main_part.step', 'partFileBase64': part, **over}


def _wait_done(client, run_id, timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = client.get(f'/api/mujoco/status?id={run_id}').json()
        if st['state'] != 'running':
            return st
        time.sleep(0.5)
    raise AssertionError('job did not finish')


def test_bad_config_stops_the_server(tmp_path):
    bad = write_config(tmp_path, place_xy=[5.0, 0.0])
    s = dataclasses.replace(load_settings(), data_dir=str(tmp_path / 'var'), config_file=bad)
    with pytest.raises(SystemExit, match='place_xy'):
        create_app(s)


def test_health_and_unknown_api(client):
    assert client.get('/api/health').json()['ok'] is True
    assert client.get('/api/nope').status_code == 404


def test_no_files_outside_the_web_app_are_served(client):
    for path in ('/etc/passwd', '/home/unbox-industry/Cad_project/config/cell.json', '/../config/cell.json',
                 '/config/cell.json', '/var/runs'):
        r = client.get(path)
        assert r.status_code == 404 or 'pick_station' not in r.text, path


@pytest.mark.parametrize('body, message', [
    ({'points': []}, 'points'),
    ({'robotId': '../x'}, 'robotId'),
    ({'partFileName': 'part.exe'}, 'Unsupported part file type'),
    ({'partFileBase64': 'not base64!'}, 'base64'),
])
def test_plan_requests_are_validated(client, body, message):
    r = client.post('/api/mujoco/auto/plan', json=_plan_body(**body))
    assert r.status_code == 400 and message in r.json()['message']


@pytest.mark.parametrize('body, message', [
    ({'runId': '../../etc', 'robotIp': 'fake'}, 'Bad run id'),
    ({'runId': 'x', 'robotIp': '1.2.3'}, 'robotIp'),
    ({'runId': 'x', 'robotIp': 'fake', 'speed': 50}, 'speed'),
    ({'runId': 'nosuchrun', 'robotIp': 'fake'}, 'No plan'),
])
def test_real_requests_are_validated(client, body, message):
    r = client.post('/api/mujoco/auto/real', json=body)
    assert r.status_code in (400, 404) and message in r.json()['message']


def test_plan_through_the_api(client):
    r = client.post('/api/mujoco/auto/plan', json=_plan_body())
    assert r.status_code == 200, r.text
    run_id = r.json()['runId']
    st = _wait_done(client, run_id)
    assert st['state'] == 'done', st['message']
    doc = client.get(f'/api/mujoco/auto/plan?id={run_id}').json()
    assert len(doc['plans']) >= 1


def test_fixture_is_served(client):
    fx = client.get('/api/mujoco/fixture').json()['fixture']
    assert fx['name'] == 'Fixture.step' and base64.b64decode(fx['base64']).startswith(b'ISO-10303-21')


def test_one_job_at_a_time_and_stop(settings):
    jobs = JobRunner(settings, REPO, module='tests.sleep_job')
    a, _, _ = jobs.new_run('a.step', b'x')
    jobs.launch(a, ['30'])
    time.sleep(0.5)
    assert jobs.running and jobs.current_run == a
    time.sleep(0.01)
    b, _, _ = jobs.new_run('b.step', b'x')
    jobs.launch(b, ['30'])   # stops job a first
    with open(os.path.join(jobs.run_dir(a), 'status.json')) as f:
        assert json.load(f) == {'state': 'failed', 'message': 'stopped'}   # job a was stopped first
    assert jobs.current_run == b and jobs.running
    assert jobs.stop() == b
    time.sleep(1.0)
    assert not jobs.running


def test_old_runs_are_pruned(settings):
    s = dataclasses.replace(settings, keep_runs=3)
    jobs = JobRunner(s, REPO)
    ids = []
    for i in range(5):
        ids.append(jobs.new_run(f'p{i}.step', b'x')[0])
        time.sleep(0.002)
    left = sorted(os.listdir(s.runs_dir))
    assert left == sorted(ids)[-3:]


def test_live_view_endpoints(client, settings):
    jobs_dir = os.path.join(settings.data_dir, 'runs', 'r1')
    os.makedirs(jobs_dir)
    assert client.get('/api/mujoco/view/scene?id=r1').status_code == 404       # job not started yet
    with open(os.path.join(jobs_dir, 'scene.json'), 'w') as f:
        json.dump({'frameFloats': 3, 'bodies': [], 'geoms': []}, f)
    with open(os.path.join(jobs_dir, 'frames.bin'), 'wb') as f:
        f.write(bytes(range(4 * 3 * 2 + 5)))                                    # 2 whole frames + a partial one
    assert client.get('/api/mujoco/view/scene?id=r1').json()['frameFloats'] == 3
    assert len(client.get('/api/mujoco/view/frames?id=r1&from=0').content) == 24
    assert len(client.get('/api/mujoco/view/frames?id=r1&from=1').content) == 12
    assert client.get('/api/mujoco/view/frames?id=../x').status_code == 400
    assert client.get('/api/mujoco/view/mesh/../../etc').status_code in (400, 404)
    assert client.get('/api/mujoco/view/mesh/' + '0' * 32).status_code == 404
