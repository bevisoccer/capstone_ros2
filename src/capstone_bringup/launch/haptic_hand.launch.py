from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    glove_node = Node(
        package='haptic_glove',
        executable='glove_node',
        name='glove_node',
        output='screen',
    )
    hand_node = Node(
        package='hand_control',
        executable='hand_node',
        name='hand_node',
        output='screen',
        parameters=[{'serial_port': 'auto'}],
    )
    haptic_bridge = Node(
        package='hand_control',
        executable='haptic_bridge_node',
        name='haptic_bridge_node',
        output='screen',
        parameters=[{'haptics_enabled': True}],
    )
    return LaunchDescription([
        glove_node,
        hand_node,
        haptic_bridge,
    ])
