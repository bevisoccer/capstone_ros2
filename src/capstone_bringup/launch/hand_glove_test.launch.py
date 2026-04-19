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
    glove_tracker = Node(
        package='orbbec_hand_tracker',
        executable='glove_tracker_node',
        name='glove_tracker_node',
        output='screen',
        additional_env={'DISPLAY': ':0'},
    )
    hand_node = Node(
        package='hand_control',
        executable='hand_node',
        name='hand_node',
        output='screen',
        parameters=[{'serial_port': '/dev/ttyACM1'}],
    )
    return LaunchDescription([
        orbbec_launch,
        glove_tracker,
        hand_node,
    ])
