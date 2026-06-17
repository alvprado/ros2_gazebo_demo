import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    velocity_controller_params = os.path.join(
        get_package_share_directory('controller'), 'config', 'velocity_controller_params.yaml')
    target_velocity_params = os.path.join(
        get_package_share_directory('controller'), 'config', 'target_velocity_params.yaml')
    simulation_params = os.path.join(
        get_package_share_directory('simulation'), 'config', 'simulation_params.yaml')

    target_velocity = Node(
        package='controller',
        executable='target_velocity',
        name='target_velocity_node',
        parameters=[target_velocity_params],
        output='screen',
    )

    velocity_controller = Node(
        package='controller',
        executable='velocity_controller',
        name='velocity_controller',
        parameters=[velocity_controller_params],
        output='screen',
    )

    robot_simulation = Node(
        package='simulation',
        executable='robot_simulation',
        name='robot_simulation',
        parameters=[simulation_params],
        output='screen',
    )

    return LaunchDescription([
        target_velocity,
        velocity_controller,
        robot_simulation,
    ])
