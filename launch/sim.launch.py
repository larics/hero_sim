"""hero_sim: Crazyflie server, ground robots, sim_attach, RViz + robot meshes from one scenario config.

    ros2 launch hero_sim sim.launch.py [config:=<name.yaml | path/to/file.yaml>]

Without config:=, $HERO_SIM_CONFIG is used (startup/_setup.sh exports it),
then config/example.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hero_sim import config


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
    if config.attach_enabled(sim):
        actions.append(Node(package='hero_sim', executable='sim_attach', name='sim_attach_node',
                            output='screen', parameters=params))
    elif sim['attach'] == 'auto':
        actions.append(LogInfo(msg='[hero_sim] hero_mrs_msgs not installed: '
                                   'sim_attach (dock ride-along, grippers) skipped'))
    if sim['rviz']:
        rviz_config = str(config.share_dir() / 'config' / 'sim.rviz')
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                            arguments=['-d', rviz_config]))
        if sim['models']:
            actions.append(Node(package='hero_sim', executable='robot_models',
                                name='robot_models', output='screen', parameters=params))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='',
                              description='Scenario YAML: a name in config/ or a path'),
        OpaqueFunction(function=_actions),
    ])
