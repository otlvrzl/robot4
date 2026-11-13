"""sim.launch.py - starts the whole project with ONE command.

A launch file lists every program (node) we want to start, plus their settings.
Without it we would have to open 5 terminals and start each program by hand.

Run it (inside Docker this is done for you by `docker compose up sim`):
    ros2 launch simple_bot sim.launch.py
Change settings on the command line:
    ros2 launch simple_bot sim.launch.py radius:=3.0 linear_speed:=0.4
Choose which driver program controls the robot:
    ros2 launch simple_bot sim.launch.py driver:=wander    # wander around a room (default)
    ros2 launch simple_bot sim.launch.py driver:=avoider   # circle + obstacle avoidance
    ros2 launch simple_bot sim.launch.py driver:=circle    # plain circle, ignores obstacles
    ros2 launch simple_bot sim.launch.py driver:=none      # no driver: steer it yourself (keyboard)
Choose the world (the file worlds/<name>.sdf):
    ros2 launch simple_bot sim.launch.py world:=room_world     # closed room with obstacles (default)
    ros2 launch simple_bot sim.launch.py world:=ground_world   # flat ground (use with the circle drivers)
"""
import os

# Helps us find the folder where our package's files were installed.
from ament_index_python.packages import get_package_share_directory
# Building blocks of a launch file.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
# Node = "start this ROS program".
from launch_ros.actions import Node
# Lets us turn a launch argument (text) into a real number for a parameter.
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ---- 1. Find our files -------------------------------------------------
    # After `colcon build`, urdf/, worlds/, ... are copied to the package's "share" folder.
    pkg = get_package_share_directory('simple_bot')
    urdf_file = os.path.join(pkg, 'urdf', 'simple_bot.urdf')            # robot description
    bridge_file = os.path.join(pkg, 'config', 'bridge.yaml')            # topic bridge settings
    rviz_dir = os.path.join(pkg, 'rviz')                                # folder with RViz layouts

    # robot_state_publisher needs the URDF as plain text, so read the whole file.
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ---- 2. Launch arguments: settings you can override on the command line ----
    radius_arg = DeclareLaunchArgument(
        'radius', default_value='5.0', description='Circle radius in metres')
    speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.3', description='Forward speed in m/s')
    # NEW: which world to load. The name must match a file in worlds/ (without ".sdf")
    # AND the <world name="..."> written inside that file.
    world_arg = DeclareLaunchArgument(
        'world', default_value='room_world',
        description="World name: 'room_world' (closed room) or 'ground_world' (flat ground)")
    # Where the robot is placed when the simulation starts (metres / radians).
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='0.0', description='Start position X in metres')
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='0.0', description='Start position Y in metres')
    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw', default_value='0.0', description='Start heading in radians (0 = along +X)')
    # NEW: which RViz layout to open (a file in rviz/). mapping.launch.py picks 'mapping.rviz'.
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='sim.rviz',
        description='RViz layout file from the rviz/ folder')
    # Which program drives the robot.
    # 'wander' (NEW) explores a room, 'avoider' and 'circle' drive circles, 'none' = nobody.
    driver_arg = DeclareLaunchArgument(
        'driver', default_value='wander',
        description="Driver to start: 'wander', 'avoider', 'circle' or 'none'")
    # NEW: how close (metres) an obstacle may get in front before the robot steers away.
    avoid_distance_arg = DeclareLaunchArgument(
        'avoid_distance', default_value='1.0',
        description='Start avoiding when an obstacle is closer than this (metres)')

    # ---- 3. Gazebo: the physics simulator ----------------------------------
    # ros_gz_sim already ships a launch file that starts Gazebo, so we include it.
    # "-r" = start the simulation running immediately (not paused).
    # The world file is built from the `world` argument: <package share>/worlds/<world>.sdf.
    # Because `world` is only known when the launch runs, we join the pieces with
    # substitutions (a list of text pieces and launch arguments) instead of an f-string.
    world_file = [os.path.join(pkg, 'worlds') + os.sep, LaunchConfiguration('world'), '.sdf']
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 3 ', *world_file]}.items())

    # ---- 4. robot_state_publisher ------------------------------------------
    # Reads the URDF and publishes the "TF tree": where every part of the robot
    # (wheels, LIDAR...) is relative to the chassis. RViz needs this to draw the robot.
    # use_sim_time=True: use the simulator's clock, not the computer's clock.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # ---- 5. Spawn the robot into Gazebo --------------------------------------
    # Gazebo starts with an empty world; this command inserts our robot model into it.
    # TimerAction waits 5 s first so Gazebo is ready, otherwise the spawn would fail.
    # -x -y -Y set the start position and heading (from the spawn_* launch arguments).
    # -z 0.08 drops the robot from a tiny height so it does not start inside the floor.
    # -world must be the same name as in the SDF file, so it comes from the `world` argument.
    spawn_robot = TimerAction(period=5.0, actions=[
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', LaunchConfiguration('world'), '-name', 'simple_bot',
                        '-topic', 'robot_description',
                        '-x', LaunchConfiguration('spawn_x'),
                        '-y', LaunchConfiguration('spawn_y'),
                        '-z', '0.08',
                        '-Y', LaunchConfiguration('spawn_yaw')])])

    # ---- 6. Bridge between Gazebo and ROS 2 --------------------------------
    # Gazebo and ROS 2 use different "message systems". The bridge translates, e.g.
    # it turns ROS /cmd_vel into a Gazebo command and Gazebo LIDAR data into ROS /scan.
    # Which topics to translate is listed in config/bridge.yaml.
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True}])

    # 6b. A second, tiny bridge only for the wheel angles. Gazebo publishes them on a topic
    #     whose name contains the world name, e.g.
    #       /world/room_world/model/simple_bot/joint_state
    #     We cannot write a changing name into bridge.yaml, so it is built here and
    #     renamed (remapped) to the standard ROS name /joint_states.
    #     robot_state_publisher needs /joint_states to show the turning wheels in RViz.
    gz_joint_topic = ['/world/', LaunchConfiguration('world'), '/model/simple_bot/joint_state']
    joint_state_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[[*gz_joint_topic, '@sensor_msgs/msg/JointState[gz.msgs.Model']],
        remappings=[(gz_joint_topic, '/joint_states')],
        parameters=[{'use_sim_time': True}])

    # ---- 7. RViz: the visualization window -----------------------------------
    # "-d" loads our saved layout so you do not have to add displays by hand.
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', [rviz_dir + os.sep, LaunchConfiguration('rviz_config')]],
        parameters=[{'use_sim_time': True}])

    # ---- 8. Our own Python nodes: they drive the robot ------------------------
    # Only ONE of them is started, chosen by the `driver` argument (see `condition=`).
    # Launch arguments are text, so ParameterValue converts them to float numbers.

    # 8a. circle_driver: the original program. Drives a circle and ignores obstacles.
    circle_driver = Node(
        package='simple_bot', executable='circle_driver',
        parameters=[{
            'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float),
            'linear_speed': ParameterValue(LaunchConfiguration('linear_speed'), value_type=float),
            'use_sim_time': True,
        }],
        output='screen',  # print this node's log messages in the terminal
        # IfCondition: start this node only if driver == 'circle'.
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('driver'), "' == 'circle'"])))

    # 8b. obstacle_avoider: NEW. Drives the circle, but reads the LIDAR (/scan) and the
    #     odometry (/odom) to steer around obstacles and then return to the circle.
    obstacle_avoider = Node(
        package='simple_bot', executable='obstacle_avoider',
        parameters=[{
            'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float),
            'linear_speed': ParameterValue(LaunchConfiguration('linear_speed'), value_type=float),
            'avoid_distance': ParameterValue(LaunchConfiguration('avoid_distance'), value_type=float),
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('driver'), "' == 'avoider'"])))

    # 8c. wander: NEW. Drives around a room and turns away from walls and obstacles
    #     using only the LIDAR (/scan). No circle. This is what we use to explore the room.
    wander = Node(
        package='simple_bot', executable='wander',
        parameters=[{
            'linear_speed': ParameterValue(LaunchConfiguration('linear_speed'), value_type=float),
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('driver'), "' == 'wander'"])))

    # ---- 9. Return the list of everything to start -----------------------------
    return LaunchDescription([
        radius_arg, speed_arg, world_arg, rviz_config_arg, driver_arg, avoid_distance_arg,
        spawn_x_arg, spawn_y_arg, spawn_yaw_arg,
        gazebo, robot_state_publisher, bridge, joint_state_bridge, spawn_robot, rviz,
        circle_driver, obstacle_avoider, wander,
    ])
