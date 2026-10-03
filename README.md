
# hero_sim

[![CI](https://github.com/KhAlamdar11/hero_sim/actions/workflows/ci.yml/badge.svg)](https://github.com/KhAlamdar11/hero_sim/actions/workflows/ci.yml)

Lightweight HERO simulation without Gazebo. Crazyflies and ground robots are simulated kinematically (velocities are integrated straight into poses), with the same ROS 2 topics and services the control stack uses.

<p align="center">
  <img src="media/rviz2.png" width="700">
</p>

## Setup

Runs inside the `icuas26_finals` docker container.

1. Mount this repo in the container. `~/dockers/icuas26_finals/first_run.sh` already has the line below; recreate the container if it was created before that line was added:
   ```bash
   --volume /home/khawaja/HERO/hero_sim:/root/ros2_ws/src/hero_sim:rw \
   ```
2. Build (inside the container):
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install --packages-select hero_sim
   source install/setup.bash
   ```

Requires `crazyflie_interfaces` (crazyswarm2) in the same workspace.

Optional: if `hero_mrs_msgs` (from hero_mrs_control_stack) is also in the workspace, the dock ride-along node (`/sim/attach_agent`) starts automatically. Without it, that node is skipped and everything else runs normally.

## Usage

```bash
cd ~/ros2_ws/src/hero_sim/startup

bash start.sh                    # default config
bash start.sh test_many.yaml     # a config from config/
bash start.sh /path/to/my.yaml   # any config file

tmux kill-session -t hero_sim    # stop
```

This opens a tmux session running the sim (CF server, ground robots, attach node, RViz) and the octomap server.

To change the default config, edit `DEFAULT_CONFIG_NAME` at the top of `startup/_setup.sh`.

## Config

One YAML file per scenario in `config/`. To make a new one, copy `config/example.yaml`, which explains every option.

```yaml
ground_robots:
  - name: scout_1
    pose: {x: 0.0, y: 0.0, yaw: 0.0}
    drive: omni                         # omni or diff
    model: scout                        # 3D model in RViz: scout or herover; leave out for TF axes only
  - name: herover_1
    pose: {x: 0.0, y: 0.6, yaw: 0.0}
    drive: omni
    model: herover

crazyflies:                             # named cf_1, cf_2, ... in this order
  - pose: {x: 1.5, y:  0.5, z: 0.03}    # cf_1
  - pose: {x: 1.5, y: -0.5, z: 0.03}    # cf_2

sim:                                    # optional, all keys have defaults
  rviz: true
  models: true                          # robot 3D models in RViz; false = TF axes only
  map: icuas26_1                        # octomap from maps/, or null for none
```

## Robot models in RViz

Crazyflies, and ground robots with a `model:`, are drawn as 3D models that follow their TF frames. To hide them, untick the **Robot models** display in RViz, or set `sim: {models: false}` to not load them at all.

The meshes in `models/<name>/` are simplified for RViz (herover 309k → 20k triangles, scout 349k → 12k). To add or re-make one, put the raw mesh in `models/source/` (not committed) and run:

```bash
blender -b --factory-startup -P models/bake_mesh.py -- \
    "models/source/herover/hero mini rover.obj" models/herover/herover.glb 6000 -90 Y
```

Robots made of several `.dae` parts (like the scout, from agilex_sitl's meshes) are first combined into one OBJ with `models/assemble_dae.py`; its header has the command. Then register the mesh in `MODEL_MESHES` in `hero_sim/config.py`.

## Driving the robots

```bash
# ground robot
ros2 topic pub /scout_1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# crazyflie
ros2 service call /cf_1/takeoff crazyflie_interfaces/srv/Takeoff "{height: 1.0, duration: {sec: 2}}"
ros2 service call /all/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2}}"
```

Ground robots hold the last `cmd_vel` until a new one arrives. Poses are published on `/<name>/pose` and as TF from `world`.