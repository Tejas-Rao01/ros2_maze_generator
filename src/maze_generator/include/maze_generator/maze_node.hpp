#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "maze_generator/maze_to_map.hpp"

class MazeNode : public rclcpp::Node {
public:
    MazeNode();
    void generateAndPublishMaze();

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    std::string maze_prefix_;
};
