from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    # load urdf, launch gazebo
    suction_ur5e_gripper_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("suction_ur5e_gripper_moveit_config"), "/launch", "/suction_ur5e_gripper_sim_control.launch.py"]
        ),
        launch_arguments={
            "launch_rviz": "true",
        }.items(),
    )

    # load moveit config
    suction_ur5e_gripper_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("suction_ur5e_gripper_moveit_config"), "/launch", "/suction_ur5e_gripper_moveit.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    suction_ur5e_gripper_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur5e_gripper_control"), "/launch", "/vision.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    nodes_to_launch = [
        suction_ur5e_gripper_control_launch,
        suction_ur5e_gripper_moveit_launch,
        suction_ur5e_gripper_vision_launch
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("suction_ur5e_gripper_moveit_config"), "config", "kinematics.yaml"]
    )

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
