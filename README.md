# hero_sim

Lightweight HERO simulation with no Gazebo. Crazyflies and ground robots are kinematic: velocities are integrated straight into poses. The ROS interface matches what `hero_mrs_control_stack` expects from the real stack.

This is the "dummy" backend of `agilex_sitl/hero_sitl` pulled out on its own. Gazebo, xacro models, CrazySim and the gz bridge are gone.

| Node | Replaces (agilex_sitl) | Does |
|------|------------------------|------|
| `crazyflie_server` | `dummy_crazyflie_server` | Every CF, with crazyswarm2's per-drone services/topics plus `/all/*` broadcasts |
| `ground_robot_sim` | `dummy_scout_node` / `dummy_herover_node` (one per robot) | All ground robots in one node, `omni` or `diff` drive |
| `sim_attach_node` | `hero_sitl/sim_attach_node.py` | `/sim/attach_agent`: a docked CF rides along on its ground robot |
| `octomap_server_node` | `icuas26_competition octomap_server.launch.py` | Serves `maps/<name>.bt` |
| `rviz2` | same | `config/sim.rviz` |

## Quick start

Everything runs in the `icuas26_finals` container. `~/dockers/icuas26_finals/first_run.sh` mounts this repo at `/root/ros2_ws/src/hero_sim`; containers created before that line was added need recreating.

```bash
# inside the container
cd ~/ros2_ws && colcon build --symlink-install --packages-select hero_sim && source install/setup.bash

cd ~/ros2_ws/src/hero_sim/startup
bash start.sh                  # DEFAULT_CONFIG_NAME from _setup.sh (test_deploy_2.yaml)
bash start.sh test_many.yaml   # bare filename -> config/test_many.yaml
bash start.sh ~/my.yaml        # any path
tmux kill-session -t hero_sim  # stop
```

`start.sh` opens a tmux session with two panes: `sim.launch.py` and `octomap.launch.py`. To change the default config, edit `DEFAULT_CONFIG_NAME` at the top of `startup/_setup.sh`.

Without tmux:

```bash
source startup/_setup.sh test_many.yaml   # exports HERO_SIM_CONFIG
ros2 launch hero_sim sim.launch.py        # or: ros2 launch hero_sim sim.launch.py config:=test_many.yaml
ros2 launch hero_sim octomap.launch.py
```

Bare config filenames given to `ros2 launch` are looked up in the *installed* `share/hero_sim/config`, so a new file needs a `colcon build` first. `start.sh` and `_setup.sh` resolve against the source `config/` directory, so new files work there without a rebuild.

## Config

One YAML per scenario in `config/`. **`config/example.yaml` documents every key.** Unknown keys are rejected at launch, so typos fail loudly.

```yaml
ground_robots:
  - name: ranger_1
    pose: {x: 0.0, y: 0.0, yaw: 0.0}
    drive: omni        # omni (default) | diff
crazyflies:            # cf_1, cf_2, ... in list order
  - pose: {x: 1.5, y: 0.5, z: 0.03}
sim:                   # optional; defaults in hero_sim/config.py SIM_DEFAULTS
  map: icuas26_1       # maps/<name>.bt, a .bt path, or null
  rviz: false
```

The scenario configs were ported from `hero_sitl/config/`. Commented-out alternatives were kept, and each active CF is tagged with its `cf_N` ID. `ranger`/`herover` both became `ground_robots`; all of them were omni in the originals.

## Interfaces

**Ground robot `<name>`**
- `/<name>/cmd_vel` (`geometry_msgs/Twist`, body frame). Held until the next message. `omni` honors `linear.x`, `linear.y` and `angular.z`; `diff` ignores `linear.y`. Planar speed is clamped to `ground_max_speed` and yaw rate to `ground_max_yaw_rate`.
- `/<name>/pose` (`PoseStamped`, frame `world`, z = 0) and TF `world -> <name>`, at `ground_rate`.

**Crazyflie `cf_N`** (same as crazyswarm2's `crazyflie_server`)
- Services: `takeoff`, `land`, `go_to`, `arm`, `notify_setpoints_stop`, `start_trajectory` (logged, not flown), `upload_trajectory`, plus sim-only `set_position` (instant teleport). Also `/all/{takeoff,land,go_to,arm,notify_setpoints_stop,start_trajectory}`.
- Command topics: `cmd_hover`, `cmd_position`, `cmd_full_state`, `cmd_velocity_world`, `cmd_vel_legacy`.
- `/cf_N/pose` and TF `world -> cf_N` at `cf_rate`; `/cf_N/status` at 1 Hz (battery drains `cf_battery_drain` V/s while flying, 4.2 V to a 3.0 V floor).

**Dock ride-along**: `/sim/attach_agent` (`hero_mrs_msgs/srv/AttachAgent`). On attach, the child's pose relative to the parent is frozen. The child is then driven through `/<child>/cmd_position` at 50 Hz until detach.

## Changes from the agilex_sitl dummy backend

- All ground robots live in one `ground_robot_sim` node (previously one `<name>_dummy_scout` node per robot).
- The yaw-rate clamp now applies to omni robots too (2 rad/s; the control stack caps at 1.5).
- Battery drain is in V/s, so it doesn't change with `cf_rate`. The default equals the old 0.0025 V/tick at 10 Hz.
- CF spawn positions are used exactly as written; hero_sitl rounded x/y to 4 decimals.
- RViz runs without `use_sim_time`, since this sim has no `/clock`.
- Octomap loads a bundled `.bt` directly, so there's no dependency on an `icuas26_*` package. `ENV_NAME`, `KEYWORD`, `COMM_RANGE` and the `GZ_*` variables are no longer exported.
- Fixed: a race where `sim_attach` could publish on a publisher being destroyed by a concurrent detach.

## Layout

```
config/      scenario YAMLs, example.yaml (reference), sim.rviz
hero_sim/    config.py, kinematics.py (no ROS), crazyflie.py, crazyflie_server.py,
             ground_robots.py, attach.py
launch/      sim.launch.py, octomap.launch.py
maps/        icuas26_1.bt, icuas26_2.bt, dummy1.bt
startup/     start.sh, _setup.sh, session.yml (tmuxinator)
test/        pytest; ROS tests skip without rclpy
```

## Tests

```bash
cd ~/ros2_ws/src/hero_sim
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is needed in the icuas26_finals image: its pip-installed `anyio` registers a pytest plugin that crashes the apt pytest at startup.
