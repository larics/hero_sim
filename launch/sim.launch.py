"""hero_sim: Crazyflie server, ground robots, sim_attach, RViz + robot meshes from one scenario config.

    ros2 launch hero_sim sim.launch.py [config:=<name.yaml | path/to/file.yaml>]

Without config:=, $HERO_SIM_CONFIG is used (startup/_setup.sh exports it),
then config/example.yaml.

With a camera on a ground robot (or sim.gazebo: true), it also starts a
headless Gazebo server on a world written to <tmp>/hero_sim/<config
name>_world.sdf (hero_sim/gazebo.py), gazebo_bridge, and with sim.gazebo_gui
Gazebo's window. RViz then gets an Image panel per camera, from a copy of
sim.rviz written to <tmp>/hero_sim/<config name>.rviz.
"""
import os
from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hero_sim import config, gazebo


def _actions(context):
    config_path = config.resolve_config(LaunchConfiguration('config').perform(context))
    scenario = config.load(config_path)
    sim = scenario.sim
    params = [{'config_path': config_path}]

    robots = ', '.join(f'{r.name}({r.drive})' for r in scenario.ground_robots) or 'none'
    actions = [
        LogInfo(msg=f'[hero_sim] config: {config_path}'),
        LogInfo(msg=f'[hero_sim] ground robots: {robots}; '
                    f'crazyflies: {len(scenario.crazyflies)}'),
        Node(package='hero_sim', executable='crazyflie_server', name='crazyflie_server',
             output='screen', parameters=params),
        Node(package='hero_sim', executable='ground_robot_sim', name='ground_robot_sim',
             output='screen', parameters=params),
    ]
    if scenario.tether is not None:
        actions.append(Node(package='hero_sim', executable='tether_sim', name='tether_sim',
                            output='screen', parameters=params))
    if config.attach_enabled(sim):
        actions.append(Node(package='hero_sim', executable='sim_attach', name='sim_attach_node',
                            output='screen', parameters=params))
    elif sim['attach'] == 'auto':
        actions.append(LogInfo(msg='[hero_sim] hero_mrs_msgs not installed: '
                                   'sim_attach (dock ride-along, grippers) skipped'))
    rviz_config = config.share_dir() / 'config' / 'sim.rviz'
    if config.gazebo_enabled(scenario):
        out = Path(tempfile.gettempdir()) / 'hero_sim' / Path(config_path).stem
        world = gazebo.write_world(f'{out}_world.sdf', scenario, config.share_dir() / 'models')
        # The map's scenery models (maps/small_house/models/), for model:// URIs.
        paths = gazebo.resource_paths(sim) + [os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
        env = {'GZ_SIM_RESOURCE_PATH': os.pathsep.join(p for p in paths if p)}
        actions += [
            LogInfo(msg=f'[hero_sim] gazebo world: {world}'),
            ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', world],
                           name='gz_server', output='screen', additional_env=env),
            Node(package='hero_sim', executable='gazebo_bridge', name='gazebo_bridge',
                 output='screen', parameters=params),
        ]
        if sim['gazebo_gui']:
            actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g'], name='gz_gui', output='screen',
                                          additional_env=env))
        if any(r.camera for r in scenario.ground_robots):
            rviz_config = Path(f'{out}.rviz')
            rviz_config.write_text(gazebo.rviz_config(
                (config.share_dir() / 'config' / 'sim.rviz').read_text(), scenario))
    if sim['rviz']:
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                            arguments=['-d', str(rviz_config)]))
        if sim['models'] or sim['map_model']:
            actions.append(Node(package='hero_sim', executable='robot_models',
                                name='robot_models', output='screen', parameters=params))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='',
                              description='Scenario YAML: a name in config/ or its folders, or a path'),
        OpaqueFunction(function=_actions),
    ])
