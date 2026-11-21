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
    with open(os.path.join(REPO, 'tests', 'fixtures', 'cad', 'main_part.step'), 'rb') as f:
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


def test_fixture_library_keeps_fixtures_and_parts(settings):
    step = base64.b64encode(b'ISO-10303-21; fake step').decode()
    with TestClient(create_app(settings)) as c:
        f1 = c.post('/api/library/fixtures', json={'name': 'F1'}).json()['fixture']
        assert c.post('/api/library/fixtures', json={'name': 'f1'}).status_code == 409      # unique names
        assert c.post('/api/library/fixtures', json={'name': '  '}).status_code == 400
        url = f"/api/library/fixtures/{f1['id']}/parts"
        p1 = c.post(url, json={'fileName': 'Part1.step', 'fileBase64': step}).json()['part']
        assert p1['name'] == 'Part1'
        assert c.post(url, json={'fileName': 'Part1.stp', 'fileBase64': step}).status_code == 409   # same name
        r = c.post(url, json={'fileName': 'part.stl', 'fileBase64': step})
        assert r.status_code == 400 and r.json()['message'] == 'Only STEP files can be added.'
        grip = {'mode': 'grip', 'points': [{'point': {'x': 1, 'y': 2, 'z': 3}, 'normal': {'x': 1, 'y': 0, 'z': 0}},
                                           {'point': {'x': -1, 'y': 2, 'z': 3}, 'normal': {'x': -1, 'y': 0, 'z': 0}}]}
        assert c.patch(f"{url}/{p1['id']}", json={'grip': grip, 'name': 'Yoke'}).json()['part']['name'] == 'Yoke'
        assert c.get(f"{url}/{p1['id']}/file").content == b'ISO-10303-21; fake step'
    # a restarted server still has them
    with TestClient(create_app(settings)) as c:
        fx = c.get('/api/library').json()['fixtures']
        assert [f['name'] for f in fx] == ['F1']
        part = fx[0]['parts'][0]
        assert part['name'] == 'Yoke' and part['grip']['points'][1]['point']['x'] == -1
        assert c.delete(f"/api/library/fixtures/{fx[0]['id']}/parts/{part['id']}").json()['ok']
        assert c.delete(f"/api/library/fixtures/{fx[0]['id']}").json()['ok']
        assert c.get('/api/library').json()['fixtures'] == []


def test_library_fixture_has_its_own_cad_file_used_for_the_view_and_the_plan(settings):
    step = base64.b64encode(b'ISO-10303-21; fixture').decode()
    with TestClient(create_app(settings)) as c:
        r = c.post('/api/library/fixtures', json={'name': 'F2', 'fileName': 'nest.txt', 'fileBase64': step})
        assert r.status_code == 400 and r.json()['message'] == 'Only STEP files can be added.'
        fx = c.post('/api/library/fixtures', json={'name': 'F2', 'fileName': 'nest.step', 'fileBase64': step}).json()['fixture']
        assert fx['file']['fileName'] == 'nest.step'
        assert c.get(f"/api/library/fixtures/{fx['id']}/file").content == b'ISO-10303-21; fixture'
        shown = c.get(f"/api/mujoco/fixture?library={fx['id']}").json()['fixture']
        assert shown['name'] == 'nest.step' and shown['seatAll'] is True
        # a fixture without its CAD file cannot be planned on
        bare = c.post('/api/library/fixtures', json={'name': 'F3'}).json()['fixture']
        r = c.post('/api/mujoco/auto/plan', json=_plan_body(fixtureId=bare['id']))
        assert r.status_code == 400 and 'no fixture CAD file' in r.json()['message']
        new = c.put(f"/api/library/fixtures/{bare['id']}/file", json={'fileName': 'n3.stp', 'fileBase64': step}).json()
        assert new['fixture']['file']['fileName'] == 'n3.stp'


def test_server_starts_when_the_fixture_cad_file_is_renamed_or_deleted(tmp_path, monkeypatch):
    """Files in cad/ get renamed and deleted: the server must start anyway."""
    import shutil
    from cell import paths
    from cell.config_check import check
    cad = tmp_path / 'cad'
    cad.mkdir()
    shutil.copy(os.path.join(REPO, 'tests', 'fixtures', 'cad', 'Fixture2.step'), cad / 'Fixture2 .step')
    with open(os.path.join(REPO, 'config', 'cell.json')) as f:
        cfg = json.load(f)
    cfg['pick_station']['file'] = str(cad / 'Fixture2.step')        # renamed with a space
    renamed = tmp_path / 'renamed.json'
    renamed.write_text(json.dumps(cfg))
    assert check(str(renamed)) == []
    ps = paths.load_config(str(renamed))['pick_station']
    assert ps['enabled'] is True and ps['file'].endswith('Fixture2 .step')
    cfg['pick_station']['file'] = str(cad / 'Gone.step')            # deleted
    gone = tmp_path / 'gone.json'
    gone.write_text(json.dumps(cfg))
    assert check(str(gone)) == []
    ps = paths.load_config(str(gone))['pick_station']
    assert ps['enabled'] is False and ps['missing_files']
