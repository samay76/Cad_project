"""The scale as the cell's fixture and part: its holes on the table holes, and the touch test (fingertips to a point)."""
import json
import os

from conftest import REPO, run_job, write_config


def test_scale_sits_on_the_four_table_holes():
    from cell.pick_station import load, place
    with open(os.path.join(REPO, 'config', 'cell.json')) as f:
        ps = json.load(f)['pick_station']
    assert ps['file'] == 'cad/scale.step'
    p = place(load(os.path.join(REPO, ps['file'])), ps['near_xy'], ps['yaw_deg'])
    assert sorted(p['table_holes']) == [[15, 0], [15, 1], [16, 0], [16, 1]]
    assert p['hole_error_mm'] <= 0.2       # CAD hole spacing 40.09 x 40.25 mm vs the 40 mm grid, centred


def test_touch_a_point_on_the_scale(tmp_path):
    cfg = write_config(tmp_path, cell_fixture=True)
    req = tmp_path / 'request.json'
    req.write_text(json.dumps({
        'partFile': os.path.join(REPO, 'cad', 'scale.step'), 'partFileName': 'scale.step', 'mode': 'touch',
        'partWorldPose': {'position': {'x': 0, 'y': 0, 'z': 0}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}},
        'robotId': 'ec66', 'gripperId': '2f85', 'tcpOffset': 145, 'preGraspDistance': 100,
        # the face of the upright that looks towards the robot (+y), 80 mm up: where the numbers are
        'points': [{'point': {'x': -28.4, 'y': -23.33, 'z': 80.0}, 'normal': {'x': 0, 'y': 1, 'z': 0}}]}))
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
