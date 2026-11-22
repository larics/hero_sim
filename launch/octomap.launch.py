"""octomap_server for the scenario's map: its obstacles, or else its sim.map.

    ros2 launch hero_sim octomap.launch.py [config:=<name.yaml | path/to/file.yaml>]

A scenario with obstacles: gets an octomap of its pillars, written to
<tmp>/hero_sim/<config name>_obstacles.bt. Otherwise sim.map is served
(config.resolve_map; null: no octomap): a .bt as it is, an image map as walls
sim.wall_height tall on its dark pixels, written to <tmp>/hero_sim/<config name>_map.bt. Either way
RViz shows it the same.

An image map is also published as a 2D map on /map by nav2's map_server
(started by its own lifecycle manager), from a copy of its .yaml written to
<tmp>/hero_sim/<config name>_map.yaml (obstacles.write_map_server_yaml).
"""
from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hero_sim import config, obstacles

# Not nav2's own names (map_server, lifecycle_manager_localization), so a nav2
# bringup can run next to the sim.
MAP_SERVER = 'sim_map_server'


def _map_server(map_yaml):
    """nav2's map_server publishing map_yaml on /map, and the lifecycle manager that starts it."""
    return [
        LogInfo(msg=f'[hero_sim] /map: {map_yaml}'),
        Node(package='nav2_map_server', executable='map_server', name=MAP_SERVER,
             output='screen',
             parameters=[{'yaml_filename': str(map_yaml), 'topic_name': 'map',
                          'frame_id': 'world'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='sim_map_lifecycle_manager', output='screen',
             parameters=[{'autostart': True, 'node_names': [MAP_SERVER]}]),
    ]


def _actions(context):
    config_path = config.resolve_config(LaunchConfiguration('config').perform(context))
    scenario = config.load(config_path)
    out = Path(tempfile.gettempdir()) / 'hero_sim' / Path(config_path).stem
    map_actions = []
    if scenario.obstacles:
        bt_file = obstacles.write_pillar_map(f'{out}_obstacles.bt', scenario.obstacles)
        source = f'{len(scenario.obstacles)} pillar(s) from obstacles:, written to {bt_file}'
    else:
        map_file = config.resolve_map(scenario.sim['map'])
        if map_file is None:
            return [LogInfo(msg='[hero_sim] no obstacles: and sim.map is null: no octomap')]
        if config.is_image_map(map_file):
            bt_file = obstacles.write_image_map(f'{out}_map.bt', map_file,
                                                scenario.sim['wall_height'])
            map_actions = _map_server(obstacles.write_map_server_yaml(f'{out}_map.yaml', map_file))
            if bt_file is None:
                return [LogInfo(msg=f'[hero_sim] {map_file} has no occupied pixels: no octomap'),
                        *map_actions]
            source = f'{map_file}, written to {bt_file}'
        else:
            bt_file = source = map_file
    return [
        LogInfo(msg=f'[hero_sim] octomap: {source}'),
        Node(package='octomap_server', executable='octomap_server_node',
             name='octomap_server_node', output='screen',
             parameters=[{'octomap_path': str(bt_file), 'frame_id': 'world'}]),
        *map_actions,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='',
                              description='Scenario YAML: a name in config/ or its folders, or a path'),
        OpaqueFunction(function=_actions),
    ])
