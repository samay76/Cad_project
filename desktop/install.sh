#!/usr/bin/env bash
# Installs the Pick Cell desktop app for this user:
#   - Electron (desktop/node_modules) and the built interface (web/dist)
#   - "Pick Cell" in the applications menu (search "Pick Cell"), and a launcher on the Desktop
# Usage:  desktop/install.sh        Then start it from the menu, or:  desktop/pick-cell
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"

echo "== Electron"
(cd "$HERE" && npm install --no-audit --no-fund)
echo "== Interface"
(cd "$REPO/web" && { [ -d node_modules ] || npm ci; } && npm run build)

chmod +x "$HERE/pick-cell"
APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
mkdir -p "$APPS"
sed -e "s#@HERE@#$HERE#g" "$HERE/pick-cell.desktop.in" > "$APPS/pick-cell.desktop"
chmod +x "$APPS/pick-cell.desktop"
update-desktop-database "$APPS" 2>/dev/null || true

DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
if [ -d "$DESK" ]; then
  cp "$APPS/pick-cell.desktop" "$DESK/pick-cell.desktop"
  chmod +x "$DESK/pick-cell.desktop"
  gio set "$DESK/pick-cell.desktop" metadata::trusted true 2>/dev/null || true
fi
echo "Installed: open \"Pick Cell\" from the applications menu (or run $HERE/pick-cell)."
