#ifndef MAZE_NODE_HPP
#define MAZE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "maze_generator/maze_to_map.hpp"

class MazeNode : public rclcpp::Node {
public:
    MazeNode();

private:
    void generateAndPublishMaze();

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    std::string maze_prefix_;
    std::unique_ptr<MazeToMap> maze_to_map_;
};

#endif // MAZE_NODE_HPP
