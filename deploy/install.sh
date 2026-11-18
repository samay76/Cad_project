#!/usr/bin/env bash
# Installs the pick cell on this PC:
#   1. the Python environment 'pick-cell' (conda / mamba, from environment.yml) - skipped if it exists
#   2. the web app: npm ci + npm run build (web/dist, served by the server)
#   3. .env from .env.example (if missing), config check, and the systemd user service
# Usage:  deploy/install.sh            (run as the user who is logged in at the cell PC)
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

CONDA="$(command -v mamba || command -v conda || true)"
[ -n "$CONDA" ] || { echo "Install miniforge first (https://github.com/conda-forge/miniforge)."; exit 1; }
ENV_PY="$("$CONDA" run -n pick-cell python -c 'import sys; print(sys.executable)' 2>/dev/null || true)"
if [ -z "$ENV_PY" ]; then
  echo "== Creating the Python environment 'pick-cell'"
  "$CONDA" env create -f environment.yml
  ENV_PY="$("$CONDA" run -n pick-cell python -c 'import sys; print(sys.executable)')"
fi
echo "== Python: $ENV_PY"

echo "== Web app"
(cd web && npm ci && npm run build)

[ -f .env ] || { cp .env.example .env; echo "== Created .env (edit it for this PC)"; }
grep -q '^CELL_PYTHON=' .env || echo "CELL_PYTHON=$ENV_PY" >> .env

echo "== Git LFS files (Table_base.step)"
git lfs pull || echo "   (git lfs not available: make sure models/Table_base.step is the real file, ~39 MB)"

echo "== Config check"
PYTHONPATH= "$ENV_PY" -m cell.config_check

echo "== systemd user service"
mkdir -p ~/.config/systemd/user
sed -e "s#@REPO@#$REPO#g" -e "s#@PYTHON@#$ENV_PY#g" deploy/pick-cell.service > ~/.config/systemd/user/pick-cell.service
systemctl --user daemon-reload
systemctl --user enable --now pick-cell
systemctl --user --no-pager status pick-cell | head -5
echo "Open http://127.0.0.1:${CELL_PORT:-8000}"
