"""octomap_server for the scenario's sim.map (maps/<name>.bt or a path; null disables it).

    ros2 launch hero_sim octomap.launch.py [config:=<name.yaml | path/to/file.yaml>]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hero_sim import config


def _actions(context):
    config_path = config.resolve_config(LaunchConfiguration('config').perform(context))
    bt_file = config.resolve_map(config.load(config_path).sim['map'])
    if bt_file is None:
        return [LogInfo(msg='[hero_sim] sim.map is null: no octomap')]
    return [
        LogInfo(msg=f'[hero_sim] octomap: {bt_file}'),
        Node(package='octomap_server', executable='octomap_server_node',
             name='octomap_server_node', output='screen',
             parameters=[{'octomap_path': bt_file, 'frame_id': 'world'}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='',
                              description='Scenario YAML: a name in config/ or a path'),
        OpaqueFunction(function=_actions),
    ])
