"""Planning and simulation of the reference pick (cad/main_part.step on the fixture), and other parts on it."""
import json
import os

import pytest

from conftest import REPO, make_request, run_job, write_config


def _plan(path):
    with open(path) as f:
        return json.load(f)


def test_grasp_test_plan_has_only_the_grasp_test_phases(grasp_test_plan):
    _, plan, _ = grasp_test_plan
    doc = _plan(plan)
    best = doc['plans'][0]
    assert best['graspTest'] is True
    names = [p['name'] for p in best['phases']]
    assert names[-7:] == ['approach', 'wait', 'grip', 'hold', 'release', 'retreat', 'home']
    assert not {'lift', 'transfer', 'place'} & set(names)
    seconds = {p['name']: p['seconds'] for p in best['phases']}
    assert seconds['wait'] == 5.0 and seconds['hold'] == 5.0


def test_full_plan_lifts_and_places(full_plan):
    _, plan, _ = full_plan
    names = [p['name'] for p in _plan(plan)['plans'][0]['phases']]
    for phase in ('approach', 'grip', 'lift', 'transfer', 'place', 'release', 'retreat', 'home'):
        assert phase in names


def test_yoke_full_pick_holds_lifts_shows_the_camera_and_bins_it(tmp_path):
    """The cell's yoke: grip, hold 5 s, lift, to the camera, turn right and left, then into the bin (sim)."""
    cfg = write_config(tmp_path, cell_fixture=True, grasp_test={'enabled': False})
    req = tmp_path / 'request.json'
    req.write_text(json.dumps({
        'partFile': os.path.join(REPO, 'cad', 'Part3.step'), 'partFileName': 'Part3.step',
        'partWorldPose': {'position': {'x': 0, 'y': 0, 'z': 0}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}},
        'robotId': 'ec66', 'gripperId': '2f85', 'tcpOffset': 145, 'preGraspDistance': 100,
        # the two faces of the -x ear (as marked in the web on 2026-11-20)
        'points': [{'point': {'x': -20.6, 'y': 0.248, 'z': 57.198}, 'normal': {'x': -1, 'y': 0, 'z': 0}},
                   {'point': {'x': -14.625, 'y': -0.639, 'z': 57.785}, 'normal': {'x': 1, 'y': 0, 'z': 0}}]}))
    plan = str(tmp_path / 'plan.json')
    code, st = run_job(['--mode', 'plan', '--config', cfg, '--request', str(req), '--plan', plan,
                        '--status', str(tmp_path / 's.json')])
    assert code == 0, st['message']
    best = _plan(plan)['plans'][0]
    names = [p['name'] for p in best['phases']]
    i = names.index('grip')
    assert names[i:i + 3] == ['grip', 'hold', 'lift'] and best['phases'][i + 1]['seconds'] == 5.0
    assert names[i + 3:i + 10] == ['to-camera', 'inspect', 'look', 'inspect', 'look', 'inspect', 'transfer']
    code, st = run_job(['--mode', 'execute', '--config', cfg, '--request', str(req), '--plan', plan,
                        '--status', str(tmp_path / 'x.json')])
    assert code == 0 and st['info']['part_in_tray'], st['message']


def test_sim_grasp_test_reaches_the_marked_points(grasp_test_plan, tmp_path):
    req, plan, cfg = grasp_test_plan
    code, st = run_job(['--mode', 'execute', '--config', cfg, '--request', req, '--plan', plan,
                        '--status', str(tmp_path / 'status.json')])
    assert code == 0 and st['ok'], st['message']
    assert st['info']['grasp_pose_error']['mm'] < 0.5
    assert max(st['info']['finger_to_point_mm']) < 0.5


def test_sim_full_pick_places_the_part_in_the_tray(full_plan, tmp_path):
    req, plan, cfg = full_plan
    code, st = run_job(['--mode', 'execute', '--config', cfg, '--request', req, '--plan', plan,
                        '--status', str(tmp_path / 'status.json')])
    assert code == 0 and st['info']['part_in_tray'], st['message']


@pytest.mark.parametrize('part', ['cad/part2.step', 'cad/Paart1.step'])
def test_other_parts_sit_on_the_fixture_and_plan(part, tmp_path):
    cfg = write_config(tmp_path)
    req = make_request(tmp_path, part=part, points=False)
    code, st = run_job(['--mode', 'plan', '--config', cfg, '--request', req, '--plan', str(tmp_path / 'plan.json'),
                        '--status', str(tmp_path / 'status.json')])
    assert code == 0 and st['state'] == 'done', st['message']
    assert st['info']['pick_station']['seated_part'] is True


def test_a_part_not_drawn_in_the_fixture_frame_is_refused(tmp_path):
    # A 20 mm cube drawn 300 mm above the fixture's origin: seated by its CAD frame it would float in the air.
    import trimesh
    cube = trimesh.creation.box(extents=(20, 20, 20))
    cube.apply_translation((0, 0, 300))
    part = tmp_path / 'floating_cube.stl'
    cube.export(str(part))
    cfg = write_config(tmp_path)
    req = make_request(tmp_path, part=str(part), points=False)
    code, st = run_job(['--mode', 'plan', '--config', cfg, '--request', req, '--plan', str(tmp_path / 'plan.json'),
                        '--status', str(tmp_path / 'status.json')])
    assert st['state'] == 'failed'
    assert "fixture's coordinate frame" in st['message'] and 'float' in st['message']
