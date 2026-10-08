# Demo media

`demo_capture.gif` (720x720, 15 fps, 176 frames, 11.7 s, ~0.6 MB) and `demo_keyframe.png` (the capture moment).

## Scenario (one recorded run, not a statistic)

| Item | Value |
|---|---|
| Launch | `pursuit.launch.py headless:=false ui:=false` |
| Agents | `num_agents:=12` |
| Strategy | `strategy:=encircle` |
| Environment | `env:=obstacle_field` (fixed hand-placed layout, 12 circles) |
| Seed | `seed:=11` |
| Episodes | `episodes_max:=1 time_limit:=25` |
| Other | params.yaml shipped values (target 1.8x, stamina on, `d_capture` 2.5, hull capture), perception=perfect, sharing_mode=legacy |
| Result | captured at t=9.73 s (a headless fast-forward trial of the same setting gave 9.93 s, so timing is not bit-exact between runs); GIF includes the 2 s result banner |

Chosen after three quick headless trials (role_encircle/seed 11: 7.57 s, encircle/seed 11: 9.93 s, role_encircle/seed 7: 4.63 s; all captured). This is selected for visibility; it says nothing about capture rate (see `log.md` for benchmarks).

## Recording method (no package source changed)

`tools/record_hook/sitecustomize.py` is picked up through `PYTHONPATH` and, only inside the `pygame_sim` process, wraps `pygame.display.flip` to save every 4th presented frame (60 fps / 4 = 15 fps) as PNG. The sim runs in real time with `SDL_VIDEODRIVER=dummy`. `tools/record_demo.sh` runs the launch and converts the frames with ffmpeg (palette GIF, 720 px wide). CPU only.

## Reproduce

pygame must be importable by `/usr/bin/python3`: `sudo apt install python3-pygame` (or `pip install --user --break-system-packages pygame`). Also needs ffmpeg.

```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash \
  && colcon build --symlink-install --base-paths src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 && cd ..
tools/record_demo.sh        # writes docs/media/demo_capture.gif (frames in out/demo_frames, git-ignored)
```
