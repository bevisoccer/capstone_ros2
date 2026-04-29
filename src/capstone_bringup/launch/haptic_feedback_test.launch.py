"""
haptic_feedback_test.launch.py

Stage 3 test: glove + robot hand + haptic feedback bridge.
No arm, no camera. Tests per-finger FSR → glove servo lock loop.

Nodes started:
  glove_node        — BLE glove sensor/servo
  hand_node         — Teensy hand (FSR + finger servos)
  haptic_bridge     — FSR threshold → glove servo lock, haptics enabled

FSR thresholds can be tuned live without restarting:
  ros2 param set /haptic_bridge_node fsr_threshold_<finger> <value>
"""

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
        parameters=[{
            'haptics_enabled':      True,
            'haptic_rate_hz':       20.0,
            'finger_gain':          1.5,
            # ── FSR lock thresholds (0.0 – 100.0 %) ──────────────────────────
            # Tune here or at runtime:
            #   ros2 param set /haptic_bridge_node fsr_threshold_<finger> <val>
            'fsr_threshold_thumb':  10.0,
            'fsr_threshold_index':  10.0,
            'fsr_threshold_middle': 10.0,
            'fsr_threshold_ring':   10.0,
            'fsr_threshold_pinky':  10.0,
        }],
    )

    return LaunchDescription([
        glove_node,
        hand_node,
        haptic_bridge,
    ])
