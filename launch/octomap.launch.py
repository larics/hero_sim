"""octomap_server for the scenario's map: its obstacles, or else its sim.map.

    ros2 launch hero_sim octomap.launch.py [config:=<name.yaml | path/to/file.yaml>]

A scenario with obstacles: gets an octomap of its pillars, written to
<tmp>/hero_sim/<config name>_obstacles.bt; otherwise sim.map is served
(maps/<name>.bt or a path; null: no octomap). Either way RViz shows it the same.
"""
from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hero_sim import config, obstacles


def _actions(context):
    config_path = config.resolve_config(LaunchConfiguration('config').perform(context))
    scenario = config.load(config_path)
    if scenario.obstacles:
        bt_file = obstacles.write_pillar_map(
            Path(tempfile.gettempdir()) / 'hero_sim' / f'{Path(config_path).stem}_obstacles.bt',
            scenario.obstacles)
        source = f'{len(scenario.obstacles)} pillar(s) from obstacles:, written to {bt_file}'
    else:
        bt_file = config.resolve_map(scenario.sim['map'])
        if bt_file is None:
            return [LogInfo(msg='[hero_sim] no obstacles: and sim.map is null: no octomap')]
        source = bt_file
    return [
        LogInfo(msg=f'[hero_sim] octomap: {source}'),
        Node(package='octomap_server', executable='octomap_server_node',
             name='octomap_server_node', output='screen',
             parameters=[{'octomap_path': str(bt_file), 'frame_id': 'world'}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='',
                              description='Scenario YAML: a name in config/ or its folders, or a path'),
        OpaqueFunction(function=_actions),
    ])
