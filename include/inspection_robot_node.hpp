// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: inspection_robot_node.hpp
// Author(s):
//
// This header defines the InspectionRobotNode class, which handles the motion
// control logic of the warehouse inspection robot. The node integrates with
// the Navigator class to compute movement commands, subscribes to detected
// damage reports, and provides a service interface to output inspection data
// as a text file.

#ifndef INSPECTION_CONTROL_NODE_HPP_
#define INSPECTION_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <memory>
#include "navigator.hpp"
#include "project_2/msg/item_entry.hpp"

#include <chrono>
using namespace std::chrono_literals;

#include "project_2/srv/get_text_file.hpp"
using project_2::srv::GetTextFile;

/**
 * @class InspectionRobotNode
 * @brief A ROS2 node responsible for controlling the warehouse inspection robot.
 *
 * This node manages robot movement by publishing velocity commands, receives
 * information about detected damages, and offers a service to export all collected
 * inspection data to a text file. It serves as the integration point between
 * perception (e.g., image processing) and navigation subsystems.
 */
class InspectionRobotNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the InspectionRobotNode class.
     *
     * Initializes publishers, subscribers, timers, and services required for
     * robot navigation and inspection data handling.
     */
    InspectionRobotNode();

    /**
     * @brief Destructor for the InspectionRobotNode class.
     */
    ~InspectionRobotNode();

private:
    /************************************************************
     ** ROS Publishers & Subscribers
     ************************************************************/

    /** 
     * @brief Publisher for velocity commands (TwistStamped).
     *
     * The published messages are used to control the robot's linear and angular
     * velocity through the `/cmd_vel` topic.
     */
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    /** 
     * @brief Timer for periodic navigation updates.
     *
     * Triggers the `update_callback()` function at a fixed interval to update
     * motion control commands and evaluate navigation goals.
     */
    rclcpp::TimerBase::SharedPtr update_timer_;

    /************************************************************
     ** Callback Functions
     ************************************************************/

    /**
     * @brief Periodic callback that computes and publishes velocity commands.
     *
     * This callback is executed at a constant rate defined by `update_timer_`.
     * It uses the `InspectionNavigator` object to compute the next motion
     * based on current state and environment data. If navigation goals are
     * met (e.g., a red surface or inspection point is detected), the robot
     * will stop moving.
     */
    void update_callback();

    /**
     * @brief Publishes a velocity command to the `/cmd_vel` topic.
     *
     * This helper function simplifies the process of sending linear and angular
     * velocity commands to the robot.
     *
     * @param linear Linear velocity in meters per second (m/s).
     * @param angular Angular velocity in radians per second (rad/s).
     */
    void update_cmd_vel(double linear, double angular);

    /**
     * @brief Callback for receiving damage detection entries.
     *
     * Subscribes to messages from the ImageProcessingNode that indicate
     * detected damaged items. Each message is appended to the internal
     * vector `DamageEntries` for later reporting or processing.
     *
     * @param entry Shared pointer to the received ItemEntry message.
     */
    void damage_callback(const project_2::msg::ItemEntry::SharedPtr entry);

    /************************************************************
     ** ROS Service Server
     ************************************************************/

    /**
     * @brief Service callback that provides a text file containing inspection data.
     *
     * Responds to `GetTextFile` service requests by generating and returning
     * a file summary of all recorded damage entries. This allows other systems
     * or nodes to retrieve inspection results.
     *
     * @param request Incoming service request (not used for input parameters).
     * @param response Service response containing the text file content or path.
     */
    void service_callback(const GetTextFile::Request::SharedPtr request,
                          const GetTextFile::Response::SharedPtr response);

    /** @brief Service server that handles GetTextFile requests. */
    rclcpp::Service<GetTextFile>::SharedPtr server;

    /************************************************************
     ** Member Variables
     ************************************************************/

    /** @brief Navigator responsible for path planning and motion decisions. */
    InspectionNavigator inspectionnavigator;

    /** @brief Counter used for assigning unique IDs to new entries or tasks. */
    int next_id_ = 1;

    /** @brief Vector storing all detected damage entries during operation. */
    std::vector<project_2::msg::ItemEntry> DamageEntries;

    /** @brief Subscriber for receiving `ItemEntry` messages from the image processor. */
    rclcpp::Subscription<project_2::msg::ItemEntry>::SharedPtr subscription_;
};

#endif  // 