#include "maze_generator/maze_node.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

MazeNode::MazeNode() : Node("maze_node") {
    this->declare_parameter<std::string>("maze_prefix", "maze");
    maze_prefix_ = this->get_parameter("maze_prefix").as_string();

    // Use Transient Local QoS for latching behavior
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("maze_map", qos);

    maze_to_map_ = std::make_unique<MazeToMap>();

    // Call it after a short delay to ensure everything is set up
    // or just call it directly if we don't mind it being synchronous in constructor.
    // Given it's a small task, calling it directly is okay, but let's use a one-shot timer
    // to keep the constructor clean and allow the node to be fully initialized.
    rclcpp::TimerBase::SharedPtr timer;
    timer = this->create_wall_timer(100ms, [this, timer]() {
        this->generateAndPublishMaze();
        timer->cancel();
    });
}

void MazeNode::generateAndPublishMaze() {
    auto map_data = maze_to_map_->get_map(maze_prefix_);
    if (map_data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load maze map from prefix: %s", maze_prefix_.c_str());
        return;
    }

    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.frame_id = "map";
    msg->header.stamp = this->now();

    msg->info.resolution = maze_to_map_->get_resolution();
    msg->info.width      = maze_to_map_->get_grid_width_cells();
    msg->info.height     = maze_to_map_->get_grid_height_cells();
    
    // Center the map roughly (can be made configurable)
    msg->info.origin.position.x = -(msg->info.width * msg->info.resolution) / 2.0;
    msg->info.origin.position.y = -(msg->info.height * msg->info.resolution) / 2.0;
    msg->info.origin.orientation.w = 1.0;

    msg->data = std::move(map_data);

    RCLCPP_INFO(this->get_logger(), "Publishing maze map (%dx%d)...", msg->info.width, msg->info.height);
    publisher_->publish(std::move(msg));
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
