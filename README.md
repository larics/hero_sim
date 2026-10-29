
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

Optional: if `hero_mrs_msgs` (from hero_mrs_control_stack) is also in the workspace, the dock ride-along node (`/sim/attach_agent`) starts automatically. Without it, that node is skipped and everything else runs normally. Called with an empty `parent_id`, the same service locks a UAV in place instead (used for a UAV hanging from a branch perch, see [Grippers](#grippers)).

## Usage

```bash
cd ~/ros2_ws/src/hero_sim/startup

bash start.sh                    # default config
bash start.sh test_many.yaml     # a config from config/, found in whichever folder it's in
bash start.sh mrs/test_many.yaml # the same, naming its folder
bash start.sh /path/to/my.yaml   # any config file

tmux kill-session -t hero_sim    # stop
```

This opens a tmux session running the sim (CF server, ground robots, attach node, RViz) and the octomap server.

To change the default config, edit `DEFAULT_CONFIG_NAME` at the top of `startup/_setup.sh`.

## Config

One YAML file per scenario in `config/`. To make a new one, copy `config/example.yaml`, which explains every option.

Scenarios can sit in `config/` itself or in a folder inside it:

| Folder | Scenarios |
|---|---|
| `config/tether/` | the tether task (`tether:` section) |
| `config/mrs/` | the control stack's, with Crazyflies |
| `config/` | everything else: the example, ground-robot-only setups |

A config is named by its filename alone (`test_many.yaml`) and found in whichever folder it's in, so filenames must be unique across folders. You can also name the folder (`mrs/test_many.yaml`) or give a path. New folders work without code changes; rebuild the package (`colcon build`) after adding or moving files.

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

## Grippers

A Crazyflie with `gripper: true` carries a two-claw gripper on top, drawn in RViz and open at the start:

```yaml
crazyflies:
  - pose: {x: 1.5, y: 0.5, z: 0.03}
    gripper: true
```

Locking the UAV in place (`/sim/attach_agent` with an empty `parent_id`, as when it hangs from a branch) closes the gripper. Detaching leaves the gripper on the branch, and the UAV flies off without it. To fit a new one:

```bash
ros2 service call /sim/cf_1/add_gripper std_srvs/srv/Trigger
```

The attach node runs the grippers, so they also need `hero_mrs_msgs`. Like the robot models, they are hidden with `sim: {models: false}`.

## Driving the robots

```bash
# ground robot
ros2 topic pub /scout_1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
ros2 service call /scout_1/go_to crazyflie_interfaces/srv/GoTo \
    "{goal: {x: 1.0, y: 0.5}, yaw: 1.57, duration: {sec: 3}}"

# crazyflie
ros2 service call /cf_1/takeoff crazyflie_interfaces/srv/Takeoff "{height: 1.0, duration: {sec: 2}}"
ros2 service call /all/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2}}"
```

Ground robots hold the last `cmd_vel` until a new one arrives. Poses are published on `/<name>/pose` and as TF from `world`.

A ground robot's `go_to` uses the same service type as the Crazyflies':

- It reaches the goal's x, y and yaw in `duration`, or more slowly if that's needed to stay within `sim.ground_max_speed` and `sim.ground_max_yaw_rate` (2 m/s and 2 rad/s by default; `duration: 0` means as fast as allowed).
- An omni robot drives straight while it turns. A diff robot turns to face the goal, drives, then turns to the yaw.
- `relative: true` makes x and y an offset from where the robot is, in the world frame; yaw stays absolute. `goal.z` and `group_mask` are ignored.
- Any `cmd_vel` message cancels it. It stops at [obstacles](#obstacles) like every other move; a blocked `go_to` waits where it stopped and carries on once the way is clear, such as when the other robot drives off.

As on hardware, where the mocap markers sit on top of the robot, a ground robot's pose is at its marker height rather than on the floor: the top of the deck, scout 0.227 m and herover 0.095 m (0 without a model). Docks the control stack places on a robot are relative to this height. To change it, set `pose: {z: ...}`; the RViz model stays on the floor.

## Obstacles

Robots can't drive or fly into obstacles. There are two kinds, and both block in 2D:

- **The map.** The octomap's voxels between 0.2 m and 2.0 m high (`sim.collision_z`) are flattened into a 2D obstacle grid, which keeps the trunks and posts and leaves out the floor, ceilings and tree canopies.
- **Pillars.** Circles listed under a top-level `obstacles:` key. They work with or without a map (`map: null`).

Each robot is a circle of its `radius`, and a move stops where that circle would touch an obstacle. The robot moves again as soon as it's commanded away. Height changes always go through, so a drone blocked by a trunk or a pillar can still climb or land, but it can't fly over it.

Ground robots also stop at each other, under the same switch. Drones aren't checked against the ground robots, so they can dock on them, or against each other. A robot with `radius: 0` ignores all of it.

```yaml
ground_robots:
  - name: scout_1
    model: scout                        # radius defaults to the model's: scout 0.42, herover 0.17, else 0.3
    radius: 0.45                        # or set it
crazyflies:
  - pose: {x: 1.5, y: 0.5, z: 0.03}     # radius 0.1 by default
obstacles:                              # optional pillars, m
  - {x: 1.5, y: 0.5, radius: 0.35}
sim:
  collision: false                      # turn all of it off: map, pillars, tether object
```

Every motion command is checked, including `go_to`, `cmd_hover`, `cmd_position` and the dock ride-along. `/<cf>/set_position` is a teleport and isn't checked. A robot that starts inside an obstacle gets a warning at launch and can move out but not further in.

Pillars are drawn in RViz as 2 m columns and published latched on `/hero_sim/obstacles` (`visualization_msgs/MarkerArray`), even with collisions off. Each pillar is a `CYLINDER` marker at its `x`, `y`, with `scale.x` its diameter:

```bash
ros2 topic echo --once /hero_sim/obstacles
```

## Tether task

A winch drags an object across the floor. The tether runs from the winch, through a pulley on a ground robot (UGV), through a ring on a Crazyflie (UAV), down to the object. A scenario sets it up with a `tether:` section; `config/tether/tether_demo.yaml` is a ready one:

```bash
bash start.sh tether_demo.yaml
```

```yaml
tether:
  winch: {x: -3.0, y: 0.0, z: 0.3}      # m; z is where the tether leaves the drum (default 0.3)
  ugv: herover_1                         # pulley on its deck, at its pose z (default: the first ground robot)
  uav: cf_1                              # ring (default cf_1)
  object: {x: 0.5, y: -0.5}              # start, on the floor
  object_size: 0.3                       # m, cube edge (default 0.3)
  goal: {x: 2.5, y: 1.5, radius: 0.4}    # radius defaults to 0.4
  slack: 3.0                             # m of tether beyond taut at the start (default 0.5)
  max_reel_speed: 0.5                    # m/s (default 0.5)
```

The winch pays out a length L. What the two pulleys don't use is the free tether below the UAV: l = L − |winch − UGV| − |UGV − UAV|. The model is quasi-static (no swing):

- While the tether is slack, the object stays where it is.
- Once it's taut, the object slides along the floor straight toward the point under the UAV.
- The object is never lifted. Once it is under the UAV, the winch slips instead.
- Pulled into an obstacle, a pillar or the map, the object jams where it touches it and the winch slips. A pull away from the obstacle frees it. The object is a circle of half its `object_size`, and like the robots it only collides while `sim.collision` is on (see [Obstacles](#obstacles)).
- The robots never feel the tether: moving them only drags the object or makes the winch slip. The robots stop at the same obstacles as the object, but the tether line itself passes through them.

Drive the UGV with `/<ugv>/cmd_vel` and fly the UAV with its Crazyflie services, as in [Driving the robots](#driving-the-robots). The winch and the object:

| Topic | Type | |
|---|---|---|
| `/winch/cmd_rate` | `std_msgs/Float64` | in, m/s: + pays out, − reels in. Held until the next message |
| `/winch/length` | `std_msgs/Float64` | L |
| `/tether/free_length` | `std_msgs/Float64` | l |
| `/tether/taut` | `std_msgs/Bool` | |
| `/tether/stuck` | `std_msgs/Bool` | object jammed against an obstacle |
| `/tether/object_pose` | `geometry_msgs/PoseStamped` | object center |
| `/tether/goal` | `geometry_msgs/PoseStamped` | goal center, latched |
| `/tether/goal_reached` | `std_msgs/Bool` | object center inside the goal circle |

```bash
ros2 topic pub /winch/cmd_rate std_msgs/msg/Float64 "{data: -0.3}"   # reel in
```

Without a `tether:` section, nothing extra runs.
