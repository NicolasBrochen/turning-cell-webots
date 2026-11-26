#!/usr/bin/env bash
# Builds site/: Webots HTML5 animation of the turning cell (interactive 3D, plays in any browser).
#   tools/build_site.sh            (START=0 END=239 simulated seconds by default: 2 lathes, dual-gripper swaps, crate exchange)
#   NO_RECORD=1 tools/build_site.sh  -> only redo the page layout (description, index, thumbnail)
# Publish on GitHub Pages:  git subtree push --prefix site origin gh-pages
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
. "$ROOT/tools/webots_env.sh"     # Webots for Windows when available (WEBOTS_MODE=linux forces Linux)
START=${START:-0}
END=${END:-239}
STEP=${STEP:-16}                  # time step of the recording (ms): 16 ms keeps the animation file small
if [ -z "${NO_RECORD:-}" ]; then
  prepare_run_root
  tmp="$RUN_ROOT/worlds/.site_record.wbt"
  sed "s/basicTimeStep 8/basicTimeStep $STEP/" "$ROOT/worlds/tournage.wbt" > "$tmp"
  cat >> "$tmp" <<WBT
Robot {
  name "animation_recorder"
  controller "animation_recorder"
  controllerArgs [ "../../site/turning_cell.html" "$START" "$END" ]
  supervisor TRUE
}
WBT
  rm -rf "$RUN_ROOT"/site/turning_cell.{html,css,json,w3d}
  mkdir -p "$RUN_ROOT/site" "$ROOT/site"
  run_webots "$tmp" --batch --mode=fast --minimize --stdout --stderr > "$ROOT/site/.build.log" 2>&1 || true
  rm -f "$tmp"
  for f in turning_cell.html turning_cell.json turning_cell.w3d; do
    [ -s "$RUN_ROOT/site/$f" ] || { echo "✗ site/$f missing (see site/.build.log)"; exit 1; }
  done
  if [ "$RUN_ROOT" != "$ROOT" ]; then
    rm -f "$ROOT"/site/turning_cell.{html,css,json,w3d}
    cp "$RUN_ROOT"/site/turning_cell.{html,css,json,w3d} "$ROOT/site/"
  fi
  grep -E "COLLISION|gripped|contact|failed|Crates exchanged|Traceback|Error" "$ROOT/site/.build.log" | tail -n 40 || true
fi
rm -f "$ROOT/site/.build.log"
# meshes referenced by the animation (SCHUNK EGK 40 and Haas ST-20, from STEP files): url "meshes/<name>.obj"
rm -rf "$ROOT/site/meshes"; mkdir -p "$ROOT/site/meshes"
for d in "$ROOT"/protos/meshes/*/; do cp "$d"*.obj "$ROOT/site/meshes/"; done   # egk40, haas_st20 (names must stay unique)
python3 - "$ROOT/site/turning_cell.html" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p, encoding="utf-8").read()
desc = ("<p><strong>Robotic turning cell</strong> (Webots R2025a): two CNC lathes facing each other, tended by a "
        "UR10e with a dual SCHUNK EGK 40 gripper on a 90° bent bracket and a 6-axis force/torque sensor.<br><br>"
        "Blanks from a crate, unload + load of each lathe in a single visit (dual gripper), force-guarded seating "
        "in the chuck, turning cycle, finished parts to the output crate, crate exchange by an operator with laser "
        "scanners slowing then stopping the robot.<br>"
        "Left click: orbit · right click: pan · mouse wheel: zoom · bottom bar: play / scrub the timeline.<br><br>"
        "Source code: <a href='https://github.com/NicolasBrochen/turning-cell-webots'>"
        "github.com/NicolasBrochen/turning-cell-webots</a></p>")
s = re.sub(r'<div class="description-container">.*?</div>', '<div class="description-container">' + desc + '</div>', s, flags=re.S)
s = re.sub(r"<h1>.*?</h1>", "<h1>3D animation: <strong>Robotic turning cell</strong></h1>", s, flags=re.S)
s = re.sub(r"<title>.*?</title>", "<title>Robotic turning cell (Webots)</title>", s, flags=re.S)
s = s.replace("<html xmlns=", "<html lang=\"en\" xmlns=") if 'lang="en"' not in s else s
s = s.replace("If the animation is not loading correctly", "If the animation does not load correctly")
open(p, "w", encoding="utf-8").write(s)
PY
cat > "$ROOT/site/index.html" <<'HTML'
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta http-equiv="refresh" content="0; url=turning_cell.html">
  <title>Robotic turning cell (Webots)</title>
</head>
<body>
  <p><a href="turning_cell.html">Open the 3D animation of the turning cell</a></p>
</body>
</html>
HTML
[ -s "$ROOT/docs/cell.jpg" ] && ffmpeg -loglevel error -y -i "$ROOT/docs/cell.jpg" -vf scale=768:-1 -q:v 4 "$ROOT/site/turning_cell.jpg"
touch "$ROOT/site/.nojekyll"
du -sh "$ROOT/site"
