// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: image_processing_node.hpp
// Author(s):
//
// This header defines the ImageProcessingNode class, which simulates the robot's
// image processing functionality. The node periodically generates fake "damage"
// detections (represented as ItemEntry messages) and publishes them to a topic.
// Each published message includes a randomly assigned ID and coordinates,
// emulating an image recognition or object detection system within the warehouse.

#ifndef IMAGE_PROCESSING_NODE_HPP_
#define IMAGE_PROCESSING_NODE_HPP_

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>
#include <chrono>

#include "project_2/msg/item_entry.hpp"

using namespace std::chrono_literals;

/**
 * @class ImageProcessingNode
 * @brief Simulates an image processing node that publishes fake damage detections.
 *
 * The ImageProcessingNode class periodically generates and publishes
 * `project_2::msg::ItemEntry` messages that contain randomly assigned IDs
 * and coordinates. This simulates the behavior of an image recognition system
 * detecting damaged items in a warehouse environment.
 */
class ImageProcessingNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for ImageProcessingNode.
     *
     * Initializes the publisher and sets up a timer to periodically
     * trigger the `update_callback()` method for publishing data.
     */
    ImageProcessingNode();

    /**
     * @brief Destructor for ImageProcessingNode.
     */
    ~ImageProcessingNode() {};

private:
    /**
     * @brief Periodically generates and publishes fake item entries.
     *
     * This callback function is triggered by a timer. It simulates image
     * detection by creating a `project_2::msg::ItemEntry` message with
     * randomized `id`, `row`, and `column` values, and publishes it to the
     * associated topic.
     */
    void update_callback();

    /// Timer that triggers periodic generation of fake item entries.
    rclcpp::TimerBase::SharedPtr fake_damage_timer_;

    /// Counter for assigning unique IDs to each generated item entry.
    int next_id_ = 1;

    /// Publisher for broadcasting `ItemEntry` messages to other nodes.
    rclcpp::Publisher<project_2::msg::ItemEntry>::SharedPtr publisher_;
};

#endif  // IMAGE_PROCESSING_NODE_HPP_