from setuptools import setup
import os
from glob import glob

package_name = 'ros2_m1_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='A simple ROS2 simulation for Apple M1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_publisher = ros2_m1_sim.robot_publisher:main',
            'robot_subscriber = ros2_m1_sim.robot_subscriber:main',
            'simple_simulation = ros2_m1_sim.simple_simulation:main',
            'ascii_visualizer = ros2_m1_sim.ascii_visualizer:main',
            'turtlesim_bridge = ros2_m1_sim.turtlesim_bridge:main',
            'keyboard_controller = ros2_m1_sim.keyboard_controller:main',
        ],
    },
)
