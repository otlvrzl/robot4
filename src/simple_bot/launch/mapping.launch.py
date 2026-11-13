"""mapping.launch.py - start the simulation AND SLAM, so the robot builds a map of the room.

Run it (inside Docker this is done for you by `docker compose up mapping`):
    ros2 launch simple_bot mapping.launch.py

What it starts:
  1. everything from sim.launch.py: Gazebo (the room), the robot, the bridge, the driver
     node (`wander` by default) and RViz - but with the layout mapping.rviz that shows the map;
  2. slam_toolbox: the SLAM program, with our settings from config/slam_toolbox.yaml.

Useful options (all optional):
    ros2 launch simple_bot mapping.launch.py driver:=none         # you steer with the keyboard
    ros2 launch simple_bot mapping.launch.py linear_speed:=0.2    # drive slower (often a cleaner map)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('simple_bot')

    # ---- Settings that can be changed on the command line ----
    world_arg = DeclareLaunchArgument(
        'world', default_value='room_world', description='World to map (see worlds/)')
    driver_arg = DeclareLaunchArgument(
        'driver', default_value='wander',
        description="Who drives: 'wander' (automatic) or 'none' (you, with the keyboard)")
    speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.3', description='Forward speed in m/s')

    # ---- 1. The simulation: we simply re-use sim.launch.py ----
    # Launch files can include other launch files. We pass our settings on to it and
    # ask it to open RViz with the mapping layout (it shows the /map).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'driver': LaunchConfiguration('driver'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'rviz_config': 'mapping.rviz',
        }.items())

    # ---- 2. SLAM Toolbox ----
    # slam_toolbox ships its own launch file for the "online asynchronous" mode. It starts
    # the node and switches it on (SLAM Toolbox is a "lifecycle node": it must be
    # configured and activated first; this launch file does that for us).
    # use_sim_time = use the simulator's clock, like all our other nodes.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items())

    # Wait 6 seconds before starting SLAM. The robot is spawned after 5 s; before that
    # there is no robot (and no TF data), and SLAM would only print error messages.
    slam_delayed = TimerAction(period=6.0, actions=[slam])

    return LaunchDescription([world_arg, driver_arg, speed_arg, sim, slam_delayed])
