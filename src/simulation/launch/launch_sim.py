import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller_params = os.path.join(
        get_package_share_directory('controller'), 'config', 'params.yaml')
    simulation_params = os.path.join(
        get_package_share_directory('simulation'), 'config', 'params.yaml')

    velocity_controller = Node(
        package='controller',
        executable='velocity_controller',
        name='velocity_controller',
        parameters=[controller_params],
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
        velocity_controller,
        robot_simulation,
    ])
