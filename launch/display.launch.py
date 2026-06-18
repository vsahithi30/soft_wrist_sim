from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("soft_wrist_sim"), "urdf", "soft_wrist.urdf.xacro"]
            ),
            " ur_type:=ur5e safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 tf_prefix:=",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("ur_description"), "rviz", "view_robot.rviz"])],
        ),
    ])
