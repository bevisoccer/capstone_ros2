from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch', 'gemini2.launch.py')
        )
    )
    orbbec_hand_tracker = Node(
        package='orbbec_hand_tracker',
        executable='orbbec_hand_tracker_node',
        name='orbbec_hand_tracker_node',
        output='screen',
    )
    safe_target_filter = Node(
        package='hand_control',
        executable='safe_target_filter_node',
        name='safe_target_filter_node',
        output='screen',
    )
    arm_control = Node(
        package='hand_control',
        executable='arm_control_node',
        name='arm_control_node',
        output='screen',
    )
    hand_controller = Node(
        package='hand_controller',
        executable='hand_controller_node',
        name='hand_controller_node',
        output='screen',
    )
    return LaunchDescription([
        orbbec_launch,
        orbbec_hand_tracker,
        safe_target_filter,
        arm_control,
        hand_controller,
    ])
