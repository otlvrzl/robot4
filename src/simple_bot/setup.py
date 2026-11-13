"""setup.py - tells the build tool (colcon) how to install this Python package.

You rarely need to edit this file. Two things matter:
  * data_files: copies non-Python files (launch, urdf, ...) to where ROS can find them.
  * entry_points: gives each runnable Python program a name, so that
    `ros2 run simple_bot circle_driver` works. If you add a new node, register it here.
"""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'simple_bot'

setup(
    name=package_name,
    version='0.1.0',
    # Finds the Python folder "simple_bot/" (the one containing __init__.py).
    packages=find_packages(exclude=['test']),
    data_files=[
        # These two lines register the package with ROS. Always needed.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copy our non-Python files to the "share" folder. glob() = "all files matching".
        # If you add a new folder (e.g. meshes/), add a line for it here.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # NEW: saved maps (.yaml + .pgm). Nav2 will load them from here in a later stage.
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',                  # <- put your name here
    maintainer_email='you@example.com',      # <- and your e-mail
    description='Simple differential-drive robot with a LIDAR: circle driving, obstacle avoidance and room exploration in Gazebo.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # "program name = python.module.path:function_to_start"
            'circle_driver = simple_bot.circle_driver:main',
            # NEW: drives the circle and steers around obstacles using the LIDAR.
            'obstacle_avoider = simple_bot.obstacle_avoider:main',
            # NEW: wanders around a room and avoids walls/obstacles (no circle).
            'wander = simple_bot.wander:main',
        ],
    },
)
