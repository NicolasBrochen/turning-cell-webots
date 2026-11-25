#!/usr/bin/env bash
# Runs the cell in Webots (fast mode, no rendering) for END simulated seconds and reports what the cell PLC logged:
# collisions detected by the physics engine (contacts of the robot), grips, F/T contacts, errors.
#   tools/sim_check.sh            (END=240 by default)      exit code 1 if a collision was detected
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
. "$ROOT/tools/webots_env.sh"
END=${END:-240}
prepare_run_root
tmp="$RUN_ROOT/worlds/.sim_check.wbt"
cp "$ROOT/worlds/tournage.wbt" "$tmp"
cat >> "$tmp" <<WBT
Robot {
  name "sim_check"
  controller "sim_check"
  controllerArgs [ "$END" ]
  supervisor TRUE
}
WBT
log="$ROOT/.sim_check.log"
run_webots "$tmp" --batch --mode=fast --minimize --no-rendering --stdout --stderr > "$log" 2>&1 || true
rm -f "$tmp"
grep -E "COLLISION|gripped|contact|failed|ROBOT|Crates exchanged|Traceback|Error|SIM_CHECK" "$log" | sed 's/^\[[a-z_ ]*\] //' || true
n=$(grep -c "COLLISION" "$log" || true)
echo "== $n collision(s) in $END s (full log: .sim_check.log)"
[ "$n" -eq 0 ]
