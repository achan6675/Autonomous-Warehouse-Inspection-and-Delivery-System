#include "image_processing_node.hpp"

/**
 * @brief Constructor that initializes the image processing node and sets up the timer and publisher.
 */
ImageProcessingNode::ImageProcessingNode() : Node("image_processing_node") {
    fake_damage_timer_ = this->create_wall_timer(
        5000ms, 
        std::bind(&ImageProcessingNode::update_callback, this)
    );

    publisher_ = this->create_publisher<project_2::msg::ItemEntry>("damages_found", 10);
}

/**
 * @brief Timer callback that simulates detecting random damages and publishes them.
 */
void ImageProcessingNode::update_callback() {
    project_2::msg::ItemEntry entry;

    entry.id = next_id_++;
    entry.row = std::rand() % 5;
    entry.column = std::rand() % 4;

    publisher_->publish(entry);

    RCLCPP_INFO(get_logger(), "Camera detected damage: id=%d (row=%d, column=%d)",
                entry.id, entry.row, entry.column);
}

/**
 * @brief Entry point for the image processing node executable.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessingNode>());
    rclcpp::shutdown();
    return 0;
}
