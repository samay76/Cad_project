"""Shared fixtures. Run from the repo root:  python -m pytest tests"""
import json
import os
import subprocess
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURES = os.path.join(REPO, 'tests', 'fixtures')
CONFIG = os.path.join(REPO, 'config', 'cell.json')


def run_job(args, cwd=REPO, timeout=600):
    """python -m cell.auto_pick_place <args> --headless; returns (exit code, status.json body)."""
    status = args[args.index('--status') + 1]
    proc = subprocess.run([sys.executable, '-m', 'cell.auto_pick_place', *args, '--headless'], cwd=cwd,
                          capture_output=True, text=True, timeout=timeout)
    with open(status) as f:
        return proc.returncode, json.load(f)


def write_config(tmp_path, name='cell.json', cell_fixture=False, **changes):
    """A copy of config/cell.json with top-level sections updated (e.g. grasp_test={'enabled': False}).
    The pick tests use the round fixture main_part.step was made for; cell_fixture=True uses the yoke fixture (a copy of Fixture2.step in tests/fixtures/cad, so the
    tests do not depend on what is in cad/)."""
    with open(CONFIG) as f:
        cfg = json.load(f)
    if not cell_fixture:
        cfg['pick_station'] = {**cfg['pick_station'], 'file': 'tests/fixtures/Fixture.step',
                               'seated_part': 'tests/fixtures/cad/main_part.step'}
    else:
        cfg['pick_station'] = {**cfg['pick_station'], 'enabled': True, 'file': 'tests/fixtures/cad/Fixture2.step',
                               'seat_all_parts': True}
        cfg['pick_station'].pop('seated_part', None)
    for key, value in changes.items():
        if isinstance(value, dict) and isinstance(cfg.get(key), dict):
            cfg[key] = {**cfg[key], **value}
        else:
            cfg[key] = value
    path = tmp_path / name
    path.write_text(json.dumps(cfg))
    return str(path)


def make_request(tmp_path, part='tests/fixtures/cad/main_part.step', points=True):
    with open(os.path.join(FIXTURES, 'main_part_request.json')) as f:
        req = json.load(f)
    req.pop('_comment', None)
    req['partFile'] = os.path.join(REPO, part)
    req['partFileName'] = os.path.basename(part)
    if not points:
        req['points'] = None
    path = tmp_path / f'request_{os.path.basename(part)}.json'
    path.write_text(json.dumps(req))
    return str(path)


@pytest.fixture(scope='session')
def grasp_test_plan(tmp_path_factory):
    """The reference pick of main_part planned with the grasp test on: (request, plan, config) paths."""
    d = tmp_path_factory.mktemp('grasp_test')
    cfg = write_config(d, grasp_test={'enabled': True})
    req = make_request(d)
    plan = str(d / 'plan.json')
    code, st = run_job(['--mode', 'plan', '--config', cfg, '--request', req, '--plan', plan,
                        '--status', str(d / 'status.json')])
    assert code == 0 and st['state'] == 'done', st.get('message')
    return req, plan, cfg


@pytest.fixture(scope='session')
def full_plan(tmp_path_factory):
    """The reference pick planned as a full pick -> lift -> place (grasp test off)."""
    d = tmp_path_factory.mktemp('full')
    cfg = write_config(d, grasp_test={'enabled': False})
    req = make_request(d)
    plan = str(d / 'plan.json')
    code, st = run_job(['--mode', 'plan', '--config', cfg, '--request', req, '--plan', plan,
                        '--status', str(d / 'status.json')])
    assert code == 0 and st['state'] == 'done', st.get('message')
    return req, plan, cfg
