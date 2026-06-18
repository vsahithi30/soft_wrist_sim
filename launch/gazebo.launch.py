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

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Spawn robot on top of pedestal at z=0.5, upright
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'ur_soft_wrist',
                    '-topic', 'robot_description',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.51',
                    '-R', '0',
                    '-P', '0',
                    '-Y', '0',
                ],
                output='screen'
            )
        ]
    )

    # Joint state publisher to hold arm in upright position
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'zeros': {
                'shoulder_pan_joint': 0.0,
                'shoulder_lift_joint': -1.5708,
                'elbow_joint': 1.5708,
                'wrist_1_joint': -3.1416,
                'wrist_2_joint': -1.5708,
                'wrist_3_joint': 0.0,
                'joint1': 0.0,
                'joint2': 0.0,
                'joint3': 0.0,
            }
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
    ])
