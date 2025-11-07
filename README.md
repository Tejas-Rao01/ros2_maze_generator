# Maze Generator Workspace

This repository contains a ROS 2 workspace for generating random mazes and launching them directly in **Gazebo**, along with a **TurtleBot3** robot spawned inside the maze. The generated maze is also published as an occupancy grid map on the `/maze_map` topic so that it can be used for navigation, SLAM, or path-planning experiments.

## Features

- Generate random mazes of customizable dimensions
- Launch the maze environment in **Gazebo**
- Automatically spawn a **TurtleBot3** robot inside the maze
- Publish the maze map to `/maze_map` (as `nav_msgs/OccupancyGrid`)
- Supports configurable maze generation algorithms
- Useful for robotics navigation and exploration experiments

## Dependencies

Ensure you have the following installed:

| Dependency | Version | Notes |
|-----------|---------|-------|
| ROS 2 | Humble | Required ROS version |
| TurtleBot3 | `turtlebot3`, `turtlebot3_simulations` | Used for robot model and simulation |
| Gazebo | Compatible with ROS 2 Humble | For simulation environment |

### TurtleBot3 Model Setup

Set your TurtleBot3 model (example: `burger`):

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## Getting Started

1. **Clone the repository:**
    ```bash
    git clone https://github.com/Tejas-Rao01/ros2_maze_generator.git
    cd maze_generator_ws
    ```

2. **Compile the code**
    ```bash
    colcon build
    ```

3. **Run the application:**
    ```bash
    source install/setup.bash
    ros2 launch maze_generator new_maze.launch.py 
    ```

## Contributing

Contributions are welcome! Please open issues or submit pull requests.

## License

This project is licensed under the MIT License.
