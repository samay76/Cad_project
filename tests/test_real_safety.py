"""Real-robot safety checks, run against FakeEC66 / FakeGripper (no hardware moves)."""
import json
import os
import signal
import threading

import numpy as np
import pytest

from cell import real_robot
from cell.auto_pick_place import AutoPickPlace, load_request
from cell.cad_pick_place import PlanError, Status


class _Args:
    def __init__(self, request):
        self.request, self.step, self.point1 = request, None, None
        self.point2 = self.click = self.normal = self.pick_offset = self.robot = self.gripper = None


@pytest.fixture(scope='module')
def cell_run(grasp_test_plan, tmp_path_factory):
    """(sim, doc, plan, cfg) for the planned reference pick, built headless like `--mode real` does."""
    req_path, plan_path, cfg_path = grasp_test_plan
    with open(cfg_path) as f:
        cfg = json.load(f)
    with open(plan_path) as f:
        doc = json.load(f)
    plan = doc['plans'][0]
    step, req, q = load_request(_Args(req_path))
    req = dict(req, robotId=doc['robot'], gripperId=doc['gripper'], tcpOffset=doc['tcpOffset'],
               preGraspDistance=plan['grasp']['preGraspDistance'], grasp=plan['grasp'])
    status = Status(str(tmp_path_factory.mktemp('real') / 'status.json'))
    sim = AutoPickPlace(step, req, q, cfg, status, headless=True)
    return sim, doc, plan, cfg


@pytest.fixture(autouse=True)
def fast_fake(monkeypatch):
    """The fake robot moves 20x faster than 5 % speed, so a full run takes a few seconds."""
    orig = real_robot.FakeEC66.move_joints
    monkeypatch.setattr(real_robot.FakeEC66, 'move_joints', lambda self, deg, speed: orig(self, deg, 100.0))
    monkeypatch.setattr(real_robot, 'POLL_HZ', 100)


def _run(cell_run, **kw):
    sim, doc, plan, cfg = cell_run
    sim.status.info.clear()
    return real_robot.run_on_real_robot(sim, doc, plan, 'fake', speed=kw.pop('speed', 5), **kw)


def _short_waits(plan):
    plan = json.loads(json.dumps(plan))
    for p in plan['phases']:
        if p['name'] in ('wait', 'hold'):
            p['seconds'] = 0.2
    return plan


def test_fake_run_completes_and_logs_every_stop(cell_run):
    sim, doc, plan, cfg = cell_run
    msg = real_robot.run_on_real_robot(sim, doc, _short_waits(plan), 'fake', speed=5,
                                       gripper_cfg=cfg['real_gripper'])
    assert msg.startswith('Real grasp test done')
    stops = sim.status.info['real_stops']
    assert len(stops) == sim.status.info['real_waypoints']
    assert all(s['max_err_deg'] <= real_robot.ARRIVE_TOLERANCE_DEG for s in stops)
    assert 'real_closed_width_mm' in sim.status.info and 'real_joints_at_pick_deg' in sim.status.info


def test_refuses_when_not_in_remote_mode(cell_run, monkeypatch):
    monkeypatch.setattr(real_robot.FakeEC66, 'mode', lambda self: 'TEACH')
    with pytest.raises(PlanError, match='REMOTE mode'):
        _run(cell_run)


def test_refuses_when_robot_has_an_alarm(cell_run, monkeypatch):
    monkeypatch.setattr(real_robot.FakeEC66, 'state', lambda self: 'alarm')
    with pytest.raises(PlanError, match="state is 'alarm'"):
        _run(cell_run)


def test_speed_is_capped(cell_run):
    sim, doc, plan, _ = cell_run
    sim.status.info.clear()
    real_robot.run_on_real_robot(sim, doc, _short_waits(plan), 'fake', speed=80)
    assert sim.status.info['real_robot']['speed_pct'] == real_robot.REAL_SPEED_MAX


def test_refuses_a_plan_outside_the_joint_limits(cell_run):
    sim, doc, plan, _ = cell_run
    bad = json.loads(json.dumps(plan))
    bad['phases'][-1]['q'][-1][2] = 3.5   # J3 beyond its 160 deg limit at the end
    with pytest.raises(PlanError, match='joint limit'):
        real_robot.run_on_real_robot(sim, doc, bad, 'fake', speed=5)


def test_stops_when_the_robot_does_not_reach_a_waypoint(cell_run, monkeypatch):
    orig = real_robot.FakeEC66.move_joints
    monkeypatch.setattr(real_robot.FakeEC66, 'move_joints',
                        lambda self, deg, speed: orig(self, np.asarray(deg) + [0, 0, 0, 0, 0, 3.0], 100.0))
    stopped = []
    monkeypatch.setattr(real_robot.FakeEC66, 'stop', lambda self: stopped.append(True))
    with pytest.raises(PlanError, match='away from'):
        _run(cell_run)
    assert stopped, 'a stop command must be sent'


def test_web_stop_sends_stop_to_the_robot(cell_run, monkeypatch):
    stopped = []
    orig_stop = real_robot.FakeEC66.stop
    monkeypatch.setattr(real_robot.FakeEC66, 'stop', lambda self: (stopped.append(True), orig_stop(self)))
    sim, doc, plan, _ = cell_run
    timer = threading.Timer(1.0, lambda: os.kill(os.getpid(), signal.SIGTERM))
    timer.start()
    try:
        with pytest.raises(PlanError, match='Stopped from the web app'):
            real_robot.run_on_real_robot(sim, doc, plan, 'fake', speed=5)
    finally:
        timer.cancel()
    assert stopped


def test_gripper_timeout_stops_the_run(cell_run, monkeypatch):
    sim, doc, plan, cfg = cell_run
    monkeypatch.setattr(real_robot.FakeGripper, 'done', lambda self, pos: (False, {'position': 0, 'object': 0}))
    monkeypatch.setattr(real_robot._grip, '__defaults__', (0.3,))
    with pytest.raises(PlanError, match='did not finish'):
        real_robot.run_on_real_robot(sim, doc, plan, 'fake', speed=5, gripper_cfg=cfg['real_gripper'])


def test_match_turns_follows_the_robots_wrist_turn():
    lo, hi = np.radians([-360] * 6), np.radians([360] * 6)
    wps = [{'kind': 'move', 'q': np.radians([180, -78, -75, -117, -89, 1.0])}]
    start = [180, -78, -75, 243, 271, 1.0]   # J4, J5 one full turn up on the robot
    done = real_robot.match_turns(wps, start, lo, hi)
    assert done == ['J4 +360 deg', 'J5 +360 deg']
    assert np.allclose(np.degrees(wps[0]['q']), [180, -78, -75, 243, 271, 1.0])


def test_web_view_streams_the_cell(cell_run, tmp_path):
    """The live view: scene.json lists the visible cell and frames.bin gets the moving bodies' poses."""
    from cell.web_view import WebView
    sim, doc, plan, _ = cell_run
    view = WebView(sim.model, sim.data, str(tmp_path), np.eye(4), label='test', fps=30)
    for k in range(1, 11):
        view.frame(sim.data, k / 60.0)   # 60 calls in 1/6 s at 30 fps: every second one is kept
    view.close()
    with open(tmp_path / 'scene.json') as f:
        scene = json.load(f)
    assert {'link1', 'g_base', 'cad_part'} <= set(scene['bodies'])
    assert len(scene['geoms']) > 20 and any(g['type'] == 'mesh' for g in scene['geoms'])
    data = np.fromfile(tmp_path / 'frames.bin', dtype='<f4').reshape(-1, scene['frameFloats'])
    assert len(data) == 6                     # the first frame + 5 at 30 fps
    i = scene['bodies'].index('link1')
    assert np.allclose(data[-1, 1 + 7 * i: 1 + 7 * i + 3], sim.data.xpos[sim.model.body('link1').id], atol=1e-6)
