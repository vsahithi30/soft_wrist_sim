import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('soft_wrist_sim')
    world_file = os.path.join(pkg_path, 'worlds', 'soft_wrist_world.sdf')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("soft_wrist_sim"), "urdf", "soft_wrist.urdf.xacro"]
            ),
            " safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 tf_prefix:=",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'ur_soft_wrist', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.51'],
                output='screen'
            )
        ]
    )

    load_joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen')]
    )

    load_arm_controller = TimerAction(
        period=8.0,
        actions=[ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'], output='screen')]
    )

    position_arm = TimerAction(
        period=15.0,
        actions=[ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--times', '5', '/arm_controller/joint_trajectory', 'trajectory_msgs/msg/JointTrajectory', '{"joint_names": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], "points": [{"positions": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0], "time_from_start": {"sec": 3}}]}'], output='screen')]
    )

    camera_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
    			'/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
   		 '/overhead_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
		],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_arm_controller,
        position_arm,
        camera_bridge,
    ])
