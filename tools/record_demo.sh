#!/usr/bin/env bash
# Record docs/media/demo_capture.gif. CPU only; see docs/media/README.md.
# Needs: ROS 2 Jazzy, built ros2_ws, a python with pygame on PYTHONPATH, ffmpeg.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FRAMES="${FRAMES:-$ROOT/out/demo_frames}"
mkdir -p "$ROOT/out"
rm -rf "$FRAMES"
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"
export SDL_VIDEODRIVER=dummy DEMO_FRAME_DIR="$FRAMES" DEMO_EVERY=4
export PYTHONPATH="$ROOT/tools/record_hook:${PYTHONPATH:-}"
ros2 launch boids_swarm pursuit.launch.py headless:=false ui:=false \
  num_agents:=12 strategy:=encircle seed:=11 env:=obstacle_field \
  episodes_max:=1 time_limit:=25 | tee "$FRAMES.log" | grep -E "EPISODE|SUMMARY" || true
N=$(ls "$FRAMES" | wc -l)
echo "frames: $N"
ffmpeg -y -loglevel error -framerate 15 -i "$FRAMES/f_%05d.png" \
  -vf "scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" \
  "$ROOT/docs/media/demo_capture.gif"
