# boids_turtlesim

ROS 2 Turtlesim Boids Swarm Coordinator — implementation of
[SDD v2](../../../SDD/Sdd_boids_turtlesim_v2.md).

Per-agent decentralized controllers with mutual peer perception:
Separation / Alignment / Cohesion + Boundary Avoidance + Wander,
converted to non-holonomic `Twist` commands (no-reverse, deadlock-fixed).

## Build

```bash
cd ~/final_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch boids_turtlesim boids.launch.py num_turtles:=6
# reproducible spawn positions:
ros2 launch boids_turtlesim boids.launch.py num_turtles:=8 seed:=42
```

## Live tuning (SDD §5.2)

All §5.1 parameters are runtime-reconfigurable, per controller:

```bash
ros2 param set /turtle1/boid_controller w_cohesion 0.6
ros2 param set /turtle1/boid_controller control_rate_hz 30.0   # retimes live
# or interactively:
rqt   # → Plugins → Configuration → Dynamic Reconfigure
```

Defaults live in [config/boids_params.yaml](config/boids_params.yaml).
Tuning order (§5.3): Separation+Boundary → Cohesion → Alignment → gains.

## Tests

```bash
python3 -m pytest src/boids_turtlesim/test/ -q     # pure-math behavior tests
```

## Layout

| File | SDD section |
|---|---|
| `boids_turtlesim/behaviors.py` | §4.3 behaviors, §4.5 kinematic conversion (pure math, unit-tested) |
| `boids_turtlesim/boid_controller.py` | §4.1 cache+timer control loop, §7 pseudocode, §5.2 live params |
| `boids_turtlesim/swarm_spawner.py` | §6.1 spawn N turtles inside margins |
| `launch/boids.launch.py` | §2.2 turtlesim + spawner + N controllers |
| `config/boids_params.yaml` | §5.1 parameter table defaults |
| `test/test_behaviors.py` | §8 unit checks (angle-wrap, no-reverse, v_min floor…) |
