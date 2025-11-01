cmake_minimum_required(VERSION 3.8)
project(maze_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

# --- Main Maze ROS2 Node ---
add_executable(maze_node
  src/maze_node.cpp
  src/maze_generator.cpp
  src/maze_parser.cpp
  src/maze_writer.cpp
  src/maze_to_map.cpp
)

ament_target_dependencies(maze_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# --- Standalone Maze File Generator (non-ROS executable) ---
add_executable(maze_file_generator
  src/generate_maze.cpp
  src/maze_generator.cpp
  src/maze_parser.cpp
  src/maze_writer.cpp
)

# No ROS dependencies for this one
target_include_directories(maze_file_generator PRIVATE include)

# --- Install targets ---
install(TARGETS
  maze_node
  maze_file_generator
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install launch files ---
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
