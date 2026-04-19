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
        sigterm_timeout='60',   # park takes ~10s — give plenty of room
        sigkill_timeout='30',
    )
    wrist_keyboard = Node(
        package='hand_control',
        executable='wrist_keyboard_node',
        name='wrist_keyboard_node',
        output='screen',
        prefix='xterm -display :0 -e',
        additional_env={'DISPLAY': ':0'},
    )
    hand_node = Node(
        package='hand_control',
        executable='hand_node',
        name='hand_node',
        output='screen',
        parameters=[{'serial_port': 'auto'}],
    )
    return LaunchDescription([
        orbbec_launch,
        glove_tracker,
        safe_target_filter,
        arm_control,
        wrist_keyboard,
        hand_node,
    ])
