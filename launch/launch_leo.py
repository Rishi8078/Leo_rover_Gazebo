import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Spawn position arguments
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Spawn X")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y")
    z_arg = DeclareLaunchArgument("z", default_value="0.5", description="Spawn Z")
    
    # Individual wheel control toggle
    individual_control_arg = DeclareLaunchArgument(
        "individual_control",
        default_value="false",
        description="Enable individual wheel velocity control (disables DiffDrive)"
    )

    # World file argument
    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.expanduser("~/vscode/Gazebo/worlds/simple_world.sdf"),
        description="Path to the Gazebo world file",
    )

    # Launch Gazebo with the world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    # Local Leo Description
    leo_description_path = os.path.expanduser("~/vscode/Gazebo/urdf/leo_sim.urdf.xacro")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": Command([
                    "xacro ", leo_description_path,
                    " individual_control:=", LaunchConfiguration("individual_control")
                ]),
                "use_sim_time": True,
            }
        ],
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "leo_rover",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # Bridge topics between ROS 2 and Gazebo
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/model/leo_rover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        parameters=[
            {"qos_overrides./tf_static.publisher.durability": "transient_local"}
        ],
        output="screen",
    )

    # Standard ROS 2 EKF Node (The "Real" Fusion)
    ekf_config_path = os.path.expanduser("~/vscode/Gazebo/config/ekf.yaml")
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('/odometry/filtered', '/odometry/filtered')]
    )

    # Slip ratio measurement node (run as process)
    slip_ratio_node = ExecuteProcess(
        cmd=["python3", os.path.expanduser("~/vscode/Gazebo/nodes/slip_ratio_node.py")],
        output="screen",
    )

    # Sensor fusion node for velocity estimation (run as process)
    sensor_fusion_node = ExecuteProcess(
        cmd=["python3", os.path.expanduser("~/vscode/Gazebo/nodes/sensor_fusion_node.py")],
        output="screen",
    )

    return LaunchDescription([
        x_arg, y_arg, z_arg,
        individual_control_arg,
        sim_world,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        topic_bridge,
        ekf_node,
        slip_ratio_node,
        sensor_fusion_node,
    ])
