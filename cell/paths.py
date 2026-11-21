"""
paths.py - Where the cell finds its models, CAD files and configuration, and where it writes caches and runs.

Everything is relative to the repository root, except the data folder (caches and per-run files), which can be
moved with the CELL_DATA_DIR environment variable, and the configuration file (CELL_CONFIG).
"""
import json
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS = os.path.join(REPO, 'models')                      # ec66/, robotiq_2f85/, Table_base.step
CAD = os.path.join(REPO, 'cad')                            # Fixture.step and part STEP files
CONFIG_FILE = os.environ.get('CELL_CONFIG') or os.path.join(REPO, 'config', 'cell.json')
DATA = os.environ.get('CELL_DATA_DIR') or os.path.join(REPO, 'var')
CACHE = os.path.join(DATA, 'cache')                        # converted meshes (rebuilt when missing)
RUNS = os.path.join(DATA, 'runs')                          # one folder per web request


def repo_path(path):
    """A path from the config file: absolute, or relative to the repository root."""
    return path if os.path.isabs(path) else os.path.normpath(os.path.join(REPO, path))


def _squash(name):
    return re.sub(r'\s+', '', name).lower()


def find_cad(path):
    """The file a config path names, tolerant of renames in cad/: the exact path, else a file in the same folder
    whose name matches ignoring spaces and case ("Fixture2 .step" for "cad/Fixture2.step"). None when gone."""
    full = repo_path(path)
    if os.path.exists(full):
        return full
    folder, want = os.path.dirname(full), _squash(os.path.basename(full))
    try:
        hits = [n for n in sorted(os.listdir(folder)) if _squash(n) == want]
    except OSError:
        return None
    return os.path.join(folder, hits[0]) if hits else None


def _resolve_station(cfg):
    """CAD files may be renamed or deleted in cad/: the cell must still start. A renamed file is found again; a
    missing one switches the config's default fixture off (a fixture from the fixture library still works)."""
    ps = cfg.get('pick_station')
    if not isinstance(ps, dict):
        return
    missing = []
    for key in ('file', 'seated_part'):
        v = ps.get(key)
        if not isinstance(v, str) or not v:
            continue
        found = find_cad(v)
        if found:
            ps[key] = found
        else:
            missing.append(v)
            if key == 'seated_part':
                ps.pop(key)
    if missing:
        if ps.get('file') in missing:
            ps['enabled'] = False
        ps['missing_files'] = missing
        print(f"[config] warning: pick_station file(s) not found: {', '.join(missing)}; "
              f"the default fixture is off until it is back in cad/ (fixture-library fixtures still work)",
              file=sys.stderr)


def load_config(path=None):
    with open(path or CONFIG_FILE) as f:
        cfg = json.load(f)
    _resolve_station(cfg)
    return cfg
