import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='ur5e_gripper_control',
            executable='ur5_vision.py',
            name='vision_node',
            parameters=[{
                    "use_sim_time":True,
                },
            ], 
            output='screen'
        ),
    ])