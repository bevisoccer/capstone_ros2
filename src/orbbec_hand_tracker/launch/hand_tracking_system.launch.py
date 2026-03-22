from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    orbbec_launch = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'gemini2.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch)
        ),

        Node(
            package='orbbec_hand_tracker',
            executable='orbbec_hand_tracker_node',
            name='orbbec_hand_tracker_node',
            output='screen'
        )
    ])
