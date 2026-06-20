import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller_params = os.path.join(
        get_package_share_directory('controller'), 'config', 'controller_params.yaml')
    targets_generator_params = os.path.join(
        get_package_share_directory('controller'), 'config', 'targets_generator_params.yaml')
    simulation_params = os.path.join(
        get_package_share_directory('simulation'), 'config', 'simulation_params.yaml')

    targets_generator = Node(
        package='controller',
        executable='targets_generator',
        name='targets_generator',
        parameters=[targets_generator_params],
        output='screen',
    )

    controller = Node(
        package='controller',
        executable='controller',
        name='controller',
        parameters=[controller_params],
        output='screen',
    )

    simple_simulation = Node(
        package='simulation',
        executable='simple_simulation',
        name='simple_simulation',
        parameters=[simulation_params],
        output='screen',
    )

    return LaunchDescription([
        targets_generator,
        controller,
        simple_simulation,
    ])
