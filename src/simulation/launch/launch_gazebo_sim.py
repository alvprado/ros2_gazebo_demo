import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Resolve file paths
    robot_description_pkg = get_package_share_directory('robot_description')
    controller_pkg = get_package_share_directory('controller')
    simulation_pkg = get_package_share_directory('simulation')

    urdf_file = os.path.join(robot_description_pkg, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(robot_description_pkg, 'worlds', 'demo_world.sdf')

    # Get params
    controller_params = os.path.join(controller_pkg, 'config', 'controller_params.yaml')
    targets_generator_params = os.path.join(
        controller_pkg, 'config', 'targets_generator_params.yaml')
    rviz_config = os.path.join(simulation_pkg, 'config', 'demo.rviz')

    # Process Xacro / URDF string
    robot_description = xacro.process_file(urdf_file).toxml()

    # robot_stat_publisher publishes the URDF to /robot_description and broadcast fixed-joint TFs
    # Wheel TFs come from /joint_states published by Gazebo JointStatePub plugin
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Gazebo simulation - run headless
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file],
        output='screen',
    )

    # Spawn robot - reads /robot_description and creates the robot entiy in Gazebo
    # Add a time delay to give gz-sim time to load the world
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'demo_robot',
                    '-topic', '/robot_description',
                    '-z', '0.15',  # 5 cm air gap; gravity settles onto ground cleanly
                ],
                output='screen',
            )
        ],
    )

    # ROS - Gazebo bridge
    # ROS -> Gazebo: /cmd_vel
    # Gazebo -> ROS: /odom, /joint_states, /tf
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/demo_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/demo_robot/tf', '/tf'),
        ],
        output='screen',
    )

    # RVIZ
    # Time delay to load TF
    # LIBGL_ALWAYS_SOFTWARE for rendering issues with local hardware
    rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
                output='screen',
            )
        ],
    )

    # Controllers
    # Time delay to load robot and settle it on the ground before receiving commands
    controllers = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller',
                executable='targets_generator',
                name='targets_generator',
                parameters=[targets_generator_params],
                output='screen',
            ),
            Node(
                package='controller',
                executable='controller',
                name='controller',
                parameters=[controller_params],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        ros_gz_bridge,
        rviz,
        controllers,
    ])
