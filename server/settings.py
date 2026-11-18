"""
settings.py - Machine settings of the server, from the environment (and an optional .env file in the repo root).

  CELL_HOST        address to listen on (default 127.0.0.1: only this PC; 0.0.0.0 for the cell network)
  CELL_PORT        port (default 8000)
  CELL_PYTHON      Python interpreter that runs the cell jobs (default: the one running the server)
  CELL_CONFIG      cell configuration file (default config/cell.json)
  CELL_DATA_DIR    caches and per-run folders (default var/)
  CELL_KEEP_RUNS   run folders kept; older ones are deleted when a new run starts (default 200)
  DISPLAY          X display for the MuJoCo windows (default :0)
"""
import os
import sys
from dataclasses import dataclass

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load_dotenv(path):
    """KEY=VALUE lines; values already in the environment win."""
    if not os.path.exists(path):
        return
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, value = line.split('=', 1)
            os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


@dataclass(frozen=True)
class Settings:
    host: str
    port: int
    python: str
    config_file: str
    data_dir: str
    keep_runs: int
    display: str
    web_dist: str

    @property
    def runs_dir(self):
        return os.path.join(self.data_dir, 'runs')


def load_settings():
    _load_dotenv(os.path.join(REPO, '.env'))
    env = os.environ
    return Settings(
        host=env.get('CELL_HOST', '127.0.0.1'),
        port=int(env.get('CELL_PORT', '8000')),
        python=env.get('CELL_PYTHON') or sys.executable,
        config_file=env.get('CELL_CONFIG') or os.path.join(REPO, 'config', 'cell.json'),
        data_dir=env.get('CELL_DATA_DIR') or os.path.join(REPO, 'var'),
        keep_runs=int(env.get('CELL_KEEP_RUNS', '200')),
        display=env.get('DISPLAY', ':0'),
        web_dist=os.path.join(REPO, 'web', 'dist'),
    )
