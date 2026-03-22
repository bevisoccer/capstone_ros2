from setuptools import find_packages, setup

package_name = 'keyboard_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bev',
    maintainer_email='bev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
           'keyboard_teleop_node = keyboard_teleop.keyboard_teleop_node:main',
            'workspace_trace_node = keyboard_teleop.workspace_trace_node:main',
            'workspace_side_trace_node = keyboard_teleop.workspace_side_trace_node:main',
            'workspace_dome_trace_node = keyboard_teleop.workspace_dome_trace_node:main',
        ],
    },
)
