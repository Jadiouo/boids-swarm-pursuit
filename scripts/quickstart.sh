#!/usr/bin/env bash
# One-shot setup + verification for boids-swarm-pursuit (ROS 2 Jazzy, Ubuntu 24.04).
#
#   scripts/quickstart.sh            # check env, build, pure tests, headless demo
#   SKIP_ROS_TESTS=0 scripts/quickstart.sh   # also run the real-process ROS tests (~1.5 min)
#
# Output goes to ./out/ (git-ignored): build log, test log, demo launch log.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/out"
mkdir -p "$OUT"

fail() { echo "ERROR: $*" >&2; exit 1; }

echo "[1/5] ROS 2 Jazzy"
[ -f /opt/ros/jazzy/setup.bash ] || fail "ROS 2 Jazzy not found at /opt/ros/jazzy. Install: https://docs.ros.org/en/jazzy/Installation.html (ros-jazzy-desktop; the Nav2 evader also needs ros-jazzy-navigation2)."
# ROS setup scripts read unset variables, so relax nounset around sourcing.
source /opt/ros/jazzy/setup.bash

echo "[2/5] pygame (must be importable by /usr/bin/python3)"
if ! /usr/bin/python3 -c "import pygame" 2>/dev/null; then
  cat >&2 <<'MSG'
pygame is missing. Recommended:
    sudo apt install python3-pygame
If that package is unavailable:
    pip install --user --break-system-packages pygame
Then re-run this script.
MSG
  exit 1
fi

echo "[3/5] colcon build"
cd "$ROOT/ros2_ws"
colcon build --symlink-install --base-paths src \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 >"$OUT/build.log" 2>&1 \
  || { tail -30 "$OUT/build.log"; fail "colcon build failed (full log: out/build.log)"; }
source install/setup.bash

echo "[4/5] pure tests (no ROS processes)"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q 2>&1 | tee "$OUT/pytest.log" | tail -3
if [ "${SKIP_ROS_TESTS:-1}" = "0" ]; then
  echo "      ROS integration tests"
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/boids_swarm/ros_test 2>&1 \
    | tee "$OUT/pytest_ros.log" | tail -3
fi

echo "[5/5] headless demo (one 20 s episode, 12 agents)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-201}" SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy
timeout 180 ros2 launch boids_swarm pursuit.launch.py headless:=true ui:=false \
  num_agents:=12 strategy:=encircle seed:=11 env:=obstacle_field \
  episodes_max:=1 time_limit:=20 >"$OUT/demo.log" 2>&1 \
  || { tail -30 "$OUT/demo.log"; fail "demo launch failed (full log: out/demo.log)"; }
grep -q Traceback "$OUT/demo.log" && fail "Traceback in out/demo.log"
grep -E "EPISODE|SUMMARY" "$OUT/demo.log" | tee "$OUT/demo_summary.txt" || true

echo
echo "OK. Logs in $OUT. Windowed demo:  ros2 launch boids_swarm pursuit.launch.py"
echo "(source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash first)"
