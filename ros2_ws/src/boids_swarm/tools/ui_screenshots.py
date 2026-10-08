"""Render the control panel to PNGs (docs/media/ui_*.png) without a display.

    cd ros2_ws && source install/setup.bash
    SDL_VIDEODRIVER=dummy ROS_DOMAIN_ID=<free id> \\
        python3 src/boids_swarm/tools/ui_screenshots.py <out_dir>

Builds a real PygameSimNode (same panel, same drawing code as the live
window) but starts no stack: the supervisor state and the staged edits are
set by hand so each picture shows a state worth documenting.
"""

import os
import sys
import time

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')

import rclpy  # noqa: E402

from boids_swarm import stack_config as sc  # noqa: E402
from boids_swarm import stack_supervisor as sv  # noqa: E402
from boids_swarm.pygame_sim_node import PygameSimNode  # noqa: E402


def settle(node, seconds=14.0):
    """Let the swarm move so the arena is not a dead start frame."""
    for a in node.agents:
        a.cmd = (1.2, 0.35)
    node.target.cmd = (2.6, -0.4)
    dt = 1.0 / node.fps
    for _ in range(int(seconds / dt)):
        node.sim_time += dt
        node.step_physics(dt)
        node.check_capture_and_score(dt)


def set_mode(node, mode):
    cfg = dict(node.pstate.applied, **sc.mode_preset(mode, node.available_evaders))
    node.pstate.commit(cfg)
    node.set_parameters([rclpy.parameter.Parameter('perception_mode', value=cfg['perception']),
                         rclpy.parameter.Parameter('sharing_mode', value=cfg['sharing_mode']),
                         rclpy.parameter.Parameter('evader', value=cfg['evader'])])


def fake_state(node, state, eta=0.0, elapsed=0.0, detail=''):
    s = node.supervisor
    s.state, s.detail, s.label = state, detail, 'nav2' if eta > 8 else ''
    s._eta, s._t_start = eta, time.monotonic() - elapsed
    s.stacks_started = 1
    node.hold = state != sv.RUNNING


def shot(node, path):
    node.render()
    import pygame
    pygame.image.save(node.screen, path)
    print('wrote', path)


def _fake_1080p():
    """The dummy driver reports a 1024x768 'display'; pretend to be a 1080p
    monitor so the window is sized as it would be for a real user."""
    import types
    import pygame
    real = pygame.display.Info
    pygame.display.Info = lambda: types.SimpleNamespace(
        current_w=1920, current_h=1080) if real().current_w <= 1024 \
        else real()


def main(out):
    os.makedirs(out, exist_ok=True)
    scale = os.environ.get('UI_SCALE', '1.5')
    args = ['--ros-args', '-p', 'stack_managed:=true',
            '-p', f'ui_scale:={scale}',
            '-p', 'ui_enabled:=true', '-p', 'num_agents:=8',
            '-p', 'target_stamina_enabled:=true']
    rclpy.init(args=args)
    node = PygameSimNode()
    _fake_1080p()
    node.setup_display()
    node.pstate.applied['env'] = 'obstacle_field'
    settle(node)

    fake_state(node, sv.RUNNING)
    shot(node, os.path.join(out, 'ui_mode_baseline.png'))

    set_mode(node, sc.SENSOR_ROS)
    shot(node, os.path.join(out, 'ui_mode_sensor_ros.png'))
    for tab in node.panel.tabs:
        node.panel.set_tab(tab)
        if tab == 'Scene':                       # show a pending edit
            node.pstate.handle(('restart', 'num_agents', 12))
            node.pstate.handle(('restart', 'env', 'pillar'))
        shot(node, os.path.join(out, f'ui_tab_{tab.lower()}.png'))
    node.pstate.staged.clear()
    node.panel.set_tab('Sensor')

    set_mode(node, sc.NAV2)
    fake_state(node, sv.STARTING, eta=9.0, elapsed=3.5)
    shot(node, os.path.join(out, 'ui_mode_nav2.png'))

    node.supervisor.state = sv.IDLE
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'docs/media')
