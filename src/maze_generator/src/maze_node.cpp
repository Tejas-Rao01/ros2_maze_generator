#include "maze_generator/maze_node.hpp"
#include <chrono>
#include <memory>
#include <iostream>


MazeNode::MazeNode() : Node("maze_node") {
    this->declare_parameter<std::string>(
        "maze_prefix",
        "/home/tejas/apps/maze_generator_ws/src/maze_generator/mazes/maze"
    );
    this->declare_parameter<int>("maze_width", 5);
    this->declare_parameter<int>("maze_height", 5);

    maze_prefix_ = this->get_parameter("maze_prefix").as_string();
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("maze_map", 10);

    // Start maze generation + publishing
    generateAndPublishMaze();
}

void MazeNode::generateAndPublishMaze() {
    
    MazeToMap maze_to_map;
    auto map_data_ptr = maze_to_map.get_map(maze_prefix_);
    if (!map_data_ptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load maze map!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting to publish maze map...");
    rclcpp::Rate rate(10); // 10 Hz

    while (rclcpp::ok()) {
        auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
        msg->header.frame_id = "map";
        msg->header.stamp = this->now();

        msg->info.resolution = maze_to_map.get_resolution();
        msg->info.width      = maze_to_map.get_grid_width_cells();
        msg->info.height     = maze_to_map.get_grid_height_cells();
        msg->info.origin.position.x = -20;
        msg->info.origin.position.y = -20;
        msg->info.origin.orientation.w = 1.0;

        msg->data.resize(map_data_ptr->size());

        for (size_t i = 0; i < map_data_ptr->size(); ++i) {
            msg->data[i] = static_cast<int8_t>((*map_data_ptr)[i]) * 100;
        }

        // visualize_map(*map_data_ptr, maze_to_map.get_grid_width_cells(), maze_to_map.get_grid_height_cells());
        publisher_->publish(std::move(msg));
        rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Maze map publishing stopped (Ctrl+C pressed).");
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MazeNode>();

    // spin will exit automatically on Ctrl+C (SIGINT)
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Shutting down MazeNode...");
    rclcpp::shutdown();
    return 0;
}
