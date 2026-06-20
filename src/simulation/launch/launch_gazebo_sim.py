import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # ── Resolve file paths ───────────────────────────────────────────────
    robot_description_pkg = get_package_share_directory('robot_description')
    controller_pkg = get_package_share_directory('controller')
    simulation_pkg = get_package_share_directory('simulation')

    urdf_file = os.path.join(robot_description_pkg, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(robot_description_pkg, 'worlds', 'demo_world.sdf')

    controller_params = os.path.join(controller_pkg, 'config', 'controller_params.yaml')
    targets_generator_params = os.path.join(
        controller_pkg, 'config', 'targets_generator_params.yaml')
    rviz_config = os.path.join(simulation_pkg, 'config', 'demo.rviz')

    # ── Process Xacro → URDF string ─────────────────────────────────────
    # xacro.process_file expands all macros and properties into plain URDF XML.
    robot_description = xacro.process_file(urdf_file).toxml()

    # ── 1. robot_state_publisher ─────────────────────────────────────────
    # Publishes the URDF to /robot_description and broadcasts fixed-joint TFs
    # (base_link → caster_wheel).  Wheel TFs come from /joint_states published
    # by the Gazebo JointStatePub plugin.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ── 2. Gazebo server (headless) ──────────────────────────────────────
    # -s  = server only (no GUI — avoids the OGRE2 crash in Parallels VMs)
    # -r  = start running immediately (don't wait for play to be pressed)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_file],
        output='screen',
    )

    # ── 3. Spawn robot ───────────────────────────────────────────────────
    # Reads /robot_description and creates the robot entity in Gazebo.
    # Delayed by 3 s to give gz-sim time to finish loading the world.
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

    # ── 4. ROS ↔ Gazebo bridge ───────────────────────────────────────────
    # Syntax:  /topic@ros_type]gz_type  → ROS2 publishes, Gazebo subscribes
    #          /topic@ros_type[gz_type  → Gazebo publishes, ROS2 subscribes
    #
    # /model/demo_robot/tf: the DiffDrive plugin publishes odom→base_link
    # transforms here as gz.msgs.Pose_V; bridging it to /tf lets RViz and
    # robot_state_publisher see the robot moving in the odom frame.
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

    # ── 5. RViz ──────────────────────────────────────────────────────────
    # Delayed by 8 s so the odom→base_link TF exists before RViz opens.
    # Without it, RViz can't find the Fixed Frame and renders a black screen.
    # LIBGL_ALWAYS_SOFTWARE forces Mesa CPU rendering — needed on Parallels
    # where the virtualised GPU doesn't expose full OpenGL to guest apps.
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

    # ── 5. Controller nodes ──────────────────────────────────────────────
    # Delayed by 6 s: robot spawns at 3 s and needs ~2-3 s to settle on the
    # ground before receiving velocity commands.  Sending cmd_vel while the
    # robot is still bouncing causes DART's constraint solver to blow up
    # ("invalid poses") because impact forces + wheel commands together
    # exceed what the solver can handle in one step.
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
