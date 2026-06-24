import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Resolve file paths and get params
    controller_params = os.path.join(
        get_package_share_directory('controller'), 'config', 'controller_params.yaml')
    targets_generator_params = os.path.join(
        get_package_share_directory('controller'), 'config', 'targets_generator_params.yaml')
    simulation_params = os.path.join(
        get_package_share_directory('simulation'), 'config', 'simulation_params.yaml')

    # Process Xacro / URDF string 
    xacro_file = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Setup rviz
    rviz_config = os.path.join(
        get_package_share_directory('simulation'), 'config', 'demo.rviz')

    # robot_stat_publisher publishes the URDF to /robot_description and broadcast fixed-joint TFs
    # Wheel TFs come from /joint_states published the simulation node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Run targets generator
    targets_generator = Node(
        package='controller',
        executable='targets_generator',
        name='targets_generator',
        parameters=[targets_generator_params],
        output='screen',
    )

    # Run controller
    controller = Node(
        package='controller',
        executable='controller',
        name='controller',
        parameters=[controller_params],
        output='screen',
    )

    # Run simulation
    simple_simulation = Node(
        package='simulation',
        executable='simple_simulation',
        name='simple_simulation',
        parameters=[simulation_params],
        output='screen',
    )

    # Run RVIZ
    # LIBGL_ALWAYS_SOFTWARE for rendering issues with local hardware
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        targets_generator,
        controller,
        simple_simulation,
        rviz,
    ])
