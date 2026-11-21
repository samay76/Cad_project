"""The cell's fixture (cad/Fixture2.step) and its yoke (cad/Part3.step): the fixture's holes on the table holes, the
yoke seated on it, and the touch test (fingertips to a point)."""
import json
import os

from conftest import REPO, run_job, write_config


def test_fixture_sits_on_the_four_table_holes():
    from cell.pick_station import load, place
    with open(os.path.join(REPO, 'config', 'cell.json')) as f:
        ps = json.load(f)['pick_station']
    assert ps['seat_all_parts'] is True   # every part (e.g. the yoke) sits on it in its CAD frame
    info = load(os.path.join(REPO, 'tests', 'fixtures', 'cad', 'Fixture2.step'))   # test copy: cad/ may change
    assert len(info['holes_mm']) == 4 and abs(info['hole_radius_mm'] - 3.5) < 0.01    # 4 x M6 clearance
    p = place(info, ps['near_xy'], ps['yaw_deg'])
    assert sorted(p['table_holes']) == [[15, 0], [15, 1], [16, 0], [16, 1]]
    assert p['hole_error_mm'] <= 0.2       # CAD holes exactly 40 x 40 mm, like the table grid


def test_touch_a_point_on_the_yoke(tmp_path):
    cfg = write_config(tmp_path, cell_fixture=True)
    req = tmp_path / 'request.json'
    req.write_text(json.dumps({
        'partFile': os.path.join(REPO, 'tests', 'fixtures', 'cad', 'Part3.step'), 'partFileName': 'Part3.step', 'mode': 'touch',
        'partWorldPose': {'position': {'x': 0, 'y': 0, 'z': 0}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}},
        'robotId': 'ec66', 'gripperId': '2f85', 'tcpOffset': 145, 'preGraspDistance': 100,
        # the top of the +x ear (its rounded end, highest point of the yoke): nothing above it
        'points': [{'point': {'x': 17.6, 'y': 0.0, 'z': 59.45}, 'normal': {'x': 0, 'y': 0, 'z': 1}}]}))
    plan = str(tmp_path / 'plan.json')
    code, st = run_job(['--mode', 'plan', '--config', cfg, '--request', str(req), '--plan', plan,
                        '--status', str(tmp_path / 's.json')])
    assert code == 0 and 'touch motion(s) planned' in st['message'], st['message']
    with open(plan) as f:
        best = json.load(f)['plans'][0]
    assert [p['name'] for p in best['phases']][-4:] == ['approach', 'wait', 'retreat', 'home']
    assert all(p['width'] == 0.0 for p in best['phases'])       # fingers closed throughout
    code, st = run_job(['--mode', 'execute', '--config', cfg, '--request', str(req), '--plan', plan,
                        '--status', str(tmp_path / 'x.json')])
    assert code == 0 and st['ok'], st['message']
    t = st['info']['touch']
    assert abs(t['to_surface_mm'] - best['touch']['standoffMm']) < 1.0   # stopped where planned before the face
    assert t['sideways_mm'] < 0.5                                       # on the marked point
