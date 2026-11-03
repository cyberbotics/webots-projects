#!/bin/bash
# Runs the example in real Webots R2025a (no clicking needed):
#   1. every preset scenario      (--launch / --exit-on-result)
#   2. the full feature tour      (--script=webots_scripts/full_tour.json)
#   3. the lesson window          (headless Chrome, webots_gui_check.py)
# Usage:  tests/run_webots_tests.sh            (from the project folder)
set -u
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WORLD=$ROOT/worlds/dynamic_grasping.wbt
OUT=$(mktemp -d)
FAIL=0

run_world() {            # run_world <name> <mode> <port> <controllerArgs...>
  local name=$1 mode=$2 port=$3; shift 3
  local args=""; for a in "$@"; do args="$args \"$a\""; done
  local tmp=$ROOT/worlds/_test_$port.wbt
  sed "s|  controller \"dynamic_grasping\"|  controller \"dynamic_grasping\"\n  controllerArgs [ $args ]|" "$WORLD" > "$tmp"
  timeout 900 webots --batch --mode="$mode" --port="$port" --minimize --stdout --stderr "$tmp" > "$OUT/$name.log" 2>&1
  local code=$?
  rm -f "$tmp" "$ROOT/worlds/._test_$port.wbproj"
  return $code
}

expect() {               # expect <name> <expected exit code> <launch>
  run_world "$1" fast 1260 "--launch=$3" "--exit-on-result"
  local code=$?
  if [ "$code" = "$2" ]; then echo "PASS  $1"; else echo "FAIL  $1 (exit $code, see $OUT/$1.log)"; FAIL=1; fi
}

echo "== Preset scenarios"
expect still_ball      0 0.55,0.1,0,0
expect gentle_roll     0 0.5,-0.8,0.2,90
expect mujoco_scene    0 0.5,-0.8,0.4,90
expect diagonal        0 0.8,-0.6,0.3,130
expect towards_robot   0 1.2,0.25,0.3,180
expect fast_pass       0 0.45,-1.0,0.6,90
expect out_of_reach    1 1.4,-1.0,0.3,90

echo "== Full feature tour"
if run_world tour fast 1261 "--script=$ROOT/tests/webots_scripts/full_tour.json"; then
  echo "PASS  full tour ($(grep -o 'all [0-9]* steps' "$OUT/tour.log"))"
else
  echo "FAIL  full tour: $(grep 'SCRIPT FAILED' "$OUT/tour.log")"; FAIL=1
fi

echo "== Lesson window (headless Chrome)"
cp "$WORLD" "$ROOT/worlds/_test_gui.wbt"
webots --batch --mode=realtime --port=1262 --minimize --stdout --stderr "$ROOT/worlds/_test_gui.wbt" > "$OUT/gui_webots.log" 2>&1 &
WEBOTS_PID=$!
google-chrome --headless=new --disable-gpu --no-sandbox --remote-debugging-port=9333 \
  --user-data-dir="$OUT/chrome" about:blank > /dev/null 2>&1 &
CHROME_PID=$!
# wait until the lesson page and Chrome's debugging port answer (up to 60 s)
for _ in $(seq 60); do
  curl -sf -o /dev/null "http://localhost:1262/robot_windows/grasping_window/grasping_window.html" &&
    curl -sf -o /dev/null http://127.0.0.1:9333/json/version && break
  sleep 1
done
sleep 3                  # let the controller finish moving home
python3 "$ROOT/tests/webots_gui_check.py" "$OUT" 1262 | tee "$OUT/gui.log" | grep -E '^FAIL|SUMMARY'
grep -q '^FAIL' "$OUT/gui.log" && FAIL=1
kill $WEBOTS_PID $CHROME_PID 2> /dev/null
sleep 1; pkill -P $WEBOTS_PID 2> /dev/null
rm -f "$ROOT/worlds/_test_gui.wbt" "$ROOT/worlds/._test_gui.wbproj"

if grep -hiE 'traceback|^error|^warning' "$OUT"/*.log | grep -v 'changed by fallback' | grep -q .; then
  echo "WARN  Webots printed errors or warnings, see $OUT"; FAIL=1
fi
echo "== Logs and screenshots: $OUT"
[ $FAIL = 0 ] && echo "ALL PASSED" || echo "SOME CHECKS FAILED"
exit $FAIL
