#!/usr/bin/env bash
# Re-records the README media from worlds/tournage.wbt (Webots renders the videos: a display is required,
# and under Windows the Webots window must stay visible while recording):
#   docs/turning_cell.mp4     one full sequence, 4x speed (overview)
#   docs/lathe_service.mp4    close-up: dual-gripper unload + load of lathe A
#   docs/crate_exchange.mp4   operator exchanging the crates, scanners slowing then stopping the robot
#   docs/cell.jpg, docs/gripper.jpg (close-up of the SCHUNK dual gripper), docs/operator.jpg   stills
#   docs/turning_cell.gif     animated preview for the README
#   tools/build_media.sh
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
. "$ROOT/tools/webots_env.sh"     # Webots Windows si disponible (WEBOTS_MODE=linux pour forcer Linux)
prepare_run_root
tmpd="$RUN_ROOT/.media_tmp"
rm -rf "$tmpd"; mkdir -p "$tmpd"
T=$(winpath "$tmpd")              # chemin des vidéos tel que Webots l'écrit
world="$RUN_ROOT/worlds/.media_record.wbt"
trap 'rm -rf "$tmpd" "$world"' EXIT
run() {  # $1 = plans.json
  # vue nommée pour le recadrage, automate en mode --sans-ihm, écran du pupitre retiré
  sed -e 's/^Viewpoint {/DEF VIEWPOINT Viewpoint {/' -e 's/^  controller "cell_plc"/  controller "cell_plc"\n  controllerArgs [ "--sans-ihm" ]/' "$ROOT/worlds/tournage.wbt" \
    | python3 -c "import re,sys; s=sys.stdin.read(); print(re.sub(r'Display \{.*?name \"IHM\".*?\n    \}', '', s, flags=re.S), end='')" > "$world"
  cat >> "$world" <<WBT
Robot { name "media_recorder" controller "media_recorder" controllerArgs [ "$(winpath "$1")" ] supervisor TRUE }
WBT
  run_webots "$world" --batch --mode=fast --stdout --stderr > "$tmpd/webots.log" 2>&1 || true
}
cat > "$tmpd/a.json" <<JSON
{"width": 1280, "height": 720, "clips": [
  {"file": "$T/cycle.mp4", "start": 0, "end": 244, "accel": 4, "position": [1.1, 3.9, 3.4], "look_at": [-0.1, -0.1, 0.9]}
]}
JSON
cat > "$tmpd/b.json" <<JSON
{"width": 1280, "height": 720, "clips": [
  {"file": "$T/zoom.mp4", "start": 48.0, "end": 53.5, "accel": 1, "position": [-0.85, 1.55, 1.35], "look_at": [-0.22, 0.85, 0.98]},
  {"file": "$T/prise.mp4", "start": 54.0, "end": 80.0, "accel": 1, "position": [-0.15, 0.75, 1.75], "look_at": [-1.15, -0.3, 1.1]},
  {"file": "$T/bande.mp4", "start": 215.0, "end": 233.0, "accel": 1, "position": [2.3, 3.4, 2.2], "look_at": [0.0, 1.1, 0.8]}
]}
JSON
run "$tmpd/a.json"
run "$tmpd/b.json"
for f in cycle.mp4 zoom.mp4 prise.mp4 bande.mp4; do
  [ -s "$tmpd/$f" ] || { tail -30 "$tmpd/webots.log"; echo "✗ $f non enregistré"; exit 1; }
done
# contrôle qualité : sous Windows, la capture vidéo devient noire si la fenêtre de Webots est masquée
# pendant l'enregistrement. Avec WEBOTS_SECOURS=linux, on refait alors les vidéos avec le Webots Linux (plus lent).
noir=""
for f in cycle.mp4 prise.mp4 bande.mp4; do
  if ffmpeg -loglevel info -i "$tmpd/$f" -vf "blackdetect=d=0.5:pix_th=0.05" -an -f null - 2>&1 | grep -q black_start; then
    noir="$noir $f"
  fi
done
if [ -n "$noir" ]; then
  if [ "$MODE" = windows ] && [ "${WEBOTS_SECOURS:-}" = linux ]; then
    echo "⚠ images noires dans$noir (fenêtre Webots masquée ?) : nouvel enregistrement avec le Webots Linux"
    rm -rf "$tmpd" "$world"; trap - EXIT
    exec env WEBOTS_MODE=linux "$0" "$@"
  fi
  echo "✗ images noires dans$noir : garder la fenêtre Webots visible pendant l'enregistrement et relancer"; exit 1
fi
D="${OUT:-$ROOT/docs}"
mkdir -p "$D"
# réencodage H.264 compact (les fichiers bruts de Webots font plusieurs dizaines de Mo)
enc() { ffmpeg -loglevel error -y -i "$1" -c:v libx264 -crf 27 -preset slow -pix_fmt yuv420p -movflags +faststart -an "$2"; }
enc "$tmpd/cycle.mp4" "$D/turning_cell.mp4"
enc "$tmpd/prise.mp4" "$D/lathe_service.mp4"
enc "$tmpd/bande.mp4" "$D/crate_exchange.mp4"
ffmpeg -loglevel error -y -ss 4.5 -i "$D/turning_cell.mp4" -frames:v 1 -q:v 3 "$D/cell.jpg"
ffmpeg -loglevel error -y -ss ${ZOOM_T:-2.5} -i "$tmpd/zoom.mp4" -frames:v 1 -q:v 3 "$D/gripper.jpg"
ffmpeg -loglevel error -y -ss 8 -i "$D/crate_exchange.mp4" -frames:v 1 -q:v 3 "$D/operator.jpg"
# aperçu animé : 24 s du cycle accéléré, 10 i/s, 640 px
ffmpeg -loglevel error -y -ss 9 -t 20 -i "$D/turning_cell.mp4" -vf "fps=10,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle" "$D/turning_cell.gif"
ls -la "$D"
