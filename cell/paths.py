"""
paths.py - Where the cell finds its models, CAD files and configuration, and where it writes caches and runs.

Everything is relative to the repository root, except the data folder (caches and per-run files), which can be
moved with the CELL_DATA_DIR environment variable, and the configuration file (CELL_CONFIG).
"""
import json
import os

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


def load_config(path=None):
    with open(path or CONFIG_FILE) as f:
        return json.load(f)
