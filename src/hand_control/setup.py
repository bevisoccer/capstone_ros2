from setuptools import setup

package_name = 'hand_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bevisoccer',
    maintainer_email='bdacosta@andrew.cmu.edu',
    description='Arm control node and motor controller for capstone robot arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_control_node = hand_control.arm_control_node:main',
            'safe_target_filter_node = hand_control.safe_target_filter_node:main',
            'wrist_keyboard_node = hand_control.wrist_keyboard_node:main',
            'hand_node = hand_control.hand_node:main',
            'haptic_bridge_node = hand_control.haptic_bridge_node:main',
        ],
    },
)
