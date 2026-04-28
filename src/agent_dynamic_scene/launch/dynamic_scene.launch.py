import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('agent_dynamic_scene')
    scenario_name = LaunchConfiguration('scenario_name').perform(context)

    scenario_map = {
        'blind_corner': 'scenario_blind_corner.yaml',
        'corridor_cross': 'scenario_corridor_cross.yaml',
    }

    if scenario_name not in scenario_map:
        raise RuntimeError(
            f'Unknown scenario_name: {scenario_name}. '
            f'Valid choices: {list(scenario_map.keys())}'
        )

    scenario_file = os.path.join(package_dir, 'config', scenario_map[scenario_name])

    return [
        Node(
            package='agent_dynamic_scene',
            executable='dynamic_actor_manager',
            name='dynamic_actor_manager',
            output='screen',
            parameters=[
                {'scenario_file': scenario_file}
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario_name',
            default_value='blind_corner',
            description='Scenario to run: blind_corner or corridor_cross'
        ),
        OpaqueFunction(function=launch_setup)
    ])
