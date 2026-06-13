# ROS 2 Maze Generator

A high-performance ROS 2 package for generating random mazes, launching them in **Gazebo Sim**, and publishing them as **OccupancyGrid** maps for navigation and SLAM experiments.

## Features

- **Efficient Generation**: Uses an iterative DFS algorithm for robust and fast maze creation.
- **Dynamic Simulation**: Automatically generates SDF world files and launches Gazebo with a TurtleBot3.
- **Occupancy Grid Integration**: Converts generated mazes into `nav_msgs/OccupancyGrid` published on `/maze_map`.
- **Latching Maps**: Uses Transient Local QoS to ensure the map is available to late-joining nodes without redundant publishing.
- **Configurable**: Fully parameterizable maze dimensions, resolution, and wall properties.
- **Portable**: No hardcoded absolute paths; works seamlessly across different environments.

## System Requirements

| Component | Requirement |
|-----------|-------------|
| **OS** | Ubuntu 22.04 (Jammy) |
| **ROS 2** | Humble Hawksbill |
| **Gazebo** | Gazebo Sim (formerly Ignition) |
| **Robot** | TurtleBot3 (burger/waffle/waffle_pi) |

## Installation

1. **Clone the Workspace:**
   ```bash
   git clone https://github.com/Tejas-Rao01/ros2_maze_generator.git
   cd ros2_maze_generator
   ```

2. **Install Dependencies:**
   Ensure you have the TurtleBot3 simulation packages installed:
   ```bash
   sudo apt update
   sudo apt install ros-humble-turtlebot3-simulations ros-humble-ros-gz
   ```

3. **Build the Package:**
   ```bash
   colcon build --packages-select maze_generator
   source install/setup.bash
   ```

## Usage

### Launching the Simulation
To generate a new maze and launch Gazebo with a TurtleBot3:
```bash
ros2 launch maze_generator new_maze.launch.py width:=15 height:=15
```

### Parameters
The launch file supports the following arguments:
- `width`: Number of cells in X direction (default: 10).
- `height`: Number of cells in Y direction (default: 10).
- `maze_prefix`: Filename prefix for generated files (stored in package share directory).

## Architecture

- **maze_file_generator**: A standalone C++ executable that handles the core logic of generating the maze structure and writing SDF/JSON metadata.
- **maze_node**: A ROS 2 node that reads maze metadata and publishes the occupancy grid map.
- **MazeToMap**: A specialized utility for efficient conversion between geometric wall data and grid-based maps.

## License

This project is licensed under the MIT License.
