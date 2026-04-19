// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_location_manager_node.hpp
// Author(s): 
//
// Manages the robot's location and delivery tasks by providing services to move the robot
// and retrieve delivery information.

#ifndef ROBOT_LOCATION_MANAGER_NODE_HPP_
#define ROBOT_LOCATION_MANAGER_NODE_HPP_

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector> 

#include "project_2/srv/move_to.hpp"
#include "project_2/srv/get_text_file.hpp"
using project_2::srv::MoveTo;
using project_2::srv::GetTextFile;

/**
 * @brief Node that manages robot location and delivery information.
 * 
 * Provides services for moving the robot to a specified location and returning
 * all stored delivery entries.
 */
class RobotLocationManagerNode : public rclcpp::Node {
public: 
    /**
     * @brief Constructor for the RobotLocationManagerNode.
     * Initializes servers and internal storage for delivery entries.
     */
    RobotLocationManagerNode();

    /**
     * @brief Destructor for the RobotLocationManagerNode.
     * Default destructor.
     */
    ~RobotLocationManagerNode(){};

private: 

    /**
     * @brief Callback for the MoveTo service.
     * Moves the robot to a requested location and sends a response.
     * @param request Shared pointer to the service request.
     * @param response Shared pointer to the service response.
     */
    void move_callback(const MoveTo::Request::SharedPtr request,
        const MoveTo::Response::SharedPtr response);

    /// Service server that handles move requests.
    rclcpp::Service<MoveTo>::SharedPtr move_server;

    /**
     * @brief Callback for the GetTextFile service.
     * Returns all stored delivery information when requested.
     * @param request Shared pointer to the service request.
     * @param response Shared pointer to the service response.
     */
    void text_callback(const GetTextFile::Request::SharedPtr request,
        const GetTextFile::Response::SharedPtr response);

    /// Service server that provides delivery information.
    rclcpp::Service<GetTextFile>::SharedPtr text_server;

    /// Vector storing all delivery entries.
    std::vector<project_2::msg::ItemEntry> DeliveryEntries;

    /// ID counter for the next delivery entry.
    int next_id_ = 1;
};

#endif
