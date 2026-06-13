#!/usr/bin/env python3

import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def run_maze_generator(context, *args, **kwargs):
    """Run the maze generator synchronously before launching other nodes."""
    maze_prefix = context.launch_configurations.get('maze_prefix', 'maze')
    
    # If it's a relative path, put it in the package's share directory or a temp dir
    if not os.path.isabs(maze_prefix):
        maze_dir = os.path.join(get_package_share_directory('maze_generator'), 'mazes')
        os.makedirs(maze_dir, exist_ok=True)
        maze_prefix = os.path.join(maze_dir, maze_prefix)

    width = context.launch_configurations.get('width', '20')
    height = context.launch_configurations.get('height', '20')
    
    print(f"--- Running maze generator: {maze_prefix} ({width}x{height}) ---")

    # Run the standalone generator executable
    # We use the executable directly from the install lib directory
    generator_exe = os.path.join(
        get_package_share_directory('maze_generator'),
        '../../lib/maze_generator/maze_file_generator'
    )
    
    # Actually, it's better to just use 'ros2 run' or find the path properly
    cmd = [
        "ros2", "run", "maze_generator", "maze_file_generator",
         maze_prefix, width, height, "1.0", "0.05", "0.2"
    ]

    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    if result.returncode == 0:
        print("--- Maze generator completed successfully. ---")
    else:
        print("--- Maze generator failed! ---")
        print(result.stderr)

    return []


def generate_launch_description():
    # --- Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.5')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='1.57')
    
    maze_prefix_config = LaunchConfiguration('maze_prefix', default='maze')
    width_config = LaunchConfiguration('width', default='10')
    height_config = LaunchConfiguration('height', default='10')

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    maze_share_dir = get_package_share_directory('maze_generator')
    
    # We'll resolve the full path to the world file
    # This is a bit tricky with OpaqueFunction because the file isn't generated yet
    # but we know where it WILL be.
    world_path = PathJoinSubstitution([
        maze_share_dir, 'mazes', 'maze.world'
    ])

    # --- Step 1: Run Maze Generator synchronously ---
    generate_maze_action = OpaqueFunction(function=run_maze_generator)

    maze_publisher_node = Node(
        package='maze_generator',
        executable='maze_node',
        name='maze_publisher',
        output='screen',
        parameters=[{
            'maze_prefix': os.path.join(maze_share_dir, 'mazes', 'maze')
        }]
    )

    # --- Step 2: Gazebo and TB3 setup ---
    # Note: We need to make sure the world path passed to gz_sim is a string
    # Since it's not generated yet, PathJoinSubstitution might be safer but gz_sim 
    # needs to be able to find it.
    
    gz_args = [
        '-r', '-v2',
        os.path.join(maze_share_dir, 'mazes', 'maze.world')
    ]

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ' '.join(gz_args), 'on_exit_shutdown': 'true'}.items()
    )

    # gzclient is usually included in gz_sim.launch.py if not -s is passed.
    # The original launch file had gzserver and gzclient separately.
    # In ROS 2 Humble / Gazebo Sim, it's often just one launch.

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
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
        DeclareLaunchArgument('maze_prefix', default_value='maze'),
        DeclareLaunchArgument('width', default_value='10'),
        DeclareLaunchArgument('height', default_value='10'),
        
        generate_maze_action,
        maze_publisher_node,
        set_env_vars_resources,
        gzserver_cmd,
        spawn_turtlebot_cmd
    ])
