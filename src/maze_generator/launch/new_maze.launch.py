#!/usr/bin/env python3

import os
import time
import subprocess
import math

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def euler_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


def run_maze_generator(context, *args, **kwargs):
    """Run the maze generator synchronously before launching other nodes."""
    maze_prefix = "/home/tejas/apps/maze_generator_ws/src/maze_generator/mazes/maze"
    maze_world = maze_prefix + ".world"

    print(f" Running maze generator for: {maze_world}")

    # Run the standalone generator executable
    cmd = [
        "ros2", "run", "maze_generator", "maze_file_generator",
         "20", "20", "1.0", "0.05", "0.2"
    ]

    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    if result.returncode == 0:
        print(" Maze generator completed successfully.")
    else:
        print(" Maze generator failed!")
        print(result.stderr)

    # Wait for the .world file to appear
    print(" Waiting for maze.world to appear...")
    timeout = 10  # seconds
    start = time.time()

    while time.time() - start < timeout:
        if os.path.exists(maze_world):
            print(" Maze file detected.")
            break
        time.sleep(1.0)
    else:
        print(" Maze file not found after timeout.")

    return []


def generate_launch_description():
    # --- Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.5')
    y_pose = LaunchConfiguration('y_pose', default='-0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='1.576')

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    maze_dir = get_package_share_directory('maze_generator')
    launch_dir = os.path.join(tb3_gazebo, 'launch')

    world = "/home/tejas/apps/maze_generator_ws/src/maze_generator/mazes/maze.world"
    # --- Step 1: Run Maze Generator synchronously ---
    generate_maze_action = OpaqueFunction(function=run_maze_generator)

    maze_publisher_action = Node(
        package='maze_generator',
        executable='maze_node',
        name='maze_publisher',
        output='screen'
    )

    # --- Step 2: Gazebo and TB3 setup ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw_pose': yaw_pose
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo, 'models')
    )

    return LaunchDescription([
        generate_maze_action,   #  Step 1: Run generator synchronously
        maze_publisher_action,
        set_env_vars_resources,
        gzserver_cmd,
        gzclient_cmd,
        spawn_turtlebot_cmd
    ])
