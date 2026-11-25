# Sélection du Webots utilisé par les scripts de construction (à « sourcer »).
#
# Sous WSL, si Webots est installé côté Windows avec un Python Windows, on l'utilise : le rendu passe directement par
# le GPU, sans la couche WSLg/D3D12 (2 à 3 fois plus rapide). Le projet est alors recopié dans un dossier Windows
# (%LOCALAPPDATA%\Temp\tournage-build), plus fiable qu'un chemin \\wsl.localhost.
#   WEBOTS_MODE=linux|windows   force un mode (par défaut : windows si disponible, sinon linux)
#   WEBOTS=...                  exécutable Webots Linux (mode linux)
#
# Fournit : RUN_ROOT (racine du projet vue par Webots), prepare_run_root, run_webots <monde> <args...>,
#           winpath <chemin> (chemin tel que Webots doit le lire)

WIN_WEBOTS="/mnt/c/Program Files/Webots/msys64/mingw64/bin/webots.exe"

_detect_mode() {
  if [ -n "${WEBOTS_MODE:-}" ]; then echo "$WEBOTS_MODE"; return; fi
  if grep -qi microsoft /proc/version 2>/dev/null && [ -x "$WIN_WEBOTS" ] && command -v cmd.exe >/dev/null; then
    echo windows
  else
    echo linux
  fi
}
MODE=$(_detect_mode)

if [ "$MODE" = windows ]; then
  LOCALAPPDATA_WIN=$(cd /mnt/c && cmd.exe /c 'echo %LOCALAPPDATA%' 2>/dev/null | tr -d '\r')
  RUN_ROOT="$(wslpath -u "$LOCALAPPDATA_WIN")/Temp/tournage-build"
  WIN_PY=$(ls -d "$(wslpath -u "$LOCALAPPDATA_WIN")"/Programs/Python/Python3* 2>/dev/null | sort -V | tail -1)
  [ -n "$WIN_PY" ] || { echo "✗ Python Windows introuvable (winget install -e --id Python.Python.3.12)"; exit 1; }
else
  RUN_ROOT="$ROOT"
fi
echo "Webots : mode $MODE (projet lu dans $RUN_ROOT)"

prepare_run_root() {
  [ "$MODE" = windows ] || return 0
  rm -rf "$RUN_ROOT"
  mkdir -p "$RUN_ROOT/site" "$RUN_ROOT/worlds"
  cp -r "$ROOT/controllers" "$ROOT/protos" "$RUN_ROOT/"
  cp "$ROOT/worlds/tournage.wbt" "$RUN_ROOT/worlds/"
  rm -rf "$RUN_ROOT"/controllers/mcp_*
  find "$RUN_ROOT" -name __pycache__ -prune -exec rm -rf {} +
  printf '@echo off\r\nset "PATH=%s;%%PATH%%"\r\n"%s" %%*\r\n' "$(wslpath -w "$WIN_PY")" "$(wslpath -w "$WIN_WEBOTS")" \
    > "$RUN_ROOT/webots_run.bat"
}

winpath() {
  if [ "$MODE" = windows ]; then wslpath -m "$1"; else echo "$1"; fi
}

run_webots() {  # run_webots <monde (chemin Linux)> [options Webots...]   (limite : WEBOTS_TIMEOUT, 1800 s)
  local world=$1; shift
  if [ "$MODE" = windows ]; then
    (cd /mnt/c && timeout "${WEBOTS_TIMEOUT:-1800}" cmd.exe /c \
      "$(wslpath -w "$RUN_ROOT/webots_run.bat") $* $(wslpath -w "$world")") | tr -d '\r'
  else
    timeout "${WEBOTS_TIMEOUT:-1800}" "${WEBOTS:-webots}" "$@" "$world"
  fi
}
