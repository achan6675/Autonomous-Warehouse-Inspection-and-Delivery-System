// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: battery_life_node.hpp
// Author(s): Amelia Chan and Imogen Coward
//
// This node monitors the robot's battery level using the /battery_state topic
// and automatically sends a recharge request to the robot_location_manager node
// when the battery falls below a safe operating threshold. It interfaces with
// the MoveTo service to command the robot to navigate to its charging station.

#ifndef BATTERY_LIFE_NODE_HPP_
#define BATTERY_LIFE_NODE_HPP_

#include <memory.h>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "project_2/srv/move_to.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "project_2/srv/get_battery.hpp"

using project_2::srv::MoveTo;
using project_2::srv::GetBattery;


/**
 * @class BatteryLifeNode
 * @brief Monitors the robot's battery level and triggers recharge behavior.
 *
 * This ROS2 node subscribes to the `/battery_state` topic and checks the
 * current battery percentage. When the percentage falls below a predefined
 * threshold, it sends a recharge request to the `robot_location_manager`
 * using the `MoveTo` service to navigate the robot to its charging station.
 */
class BatteryLifeNode : public rclcpp::Node {
public: 
    /**
     * @brief Construct a new BatteryLifeNode object.
     *
     * Initializes ROS2 interfaces including:
     *  - The service client for `MoveTo` requests.
     *  - The subscriber to `/battery_state`.
     */
    BatteryLifeNode();

    /**
     * @brief Destroy the BatteryLifeNode object.
     *
     * Default destructor. Automatically handles cleanup of ROS2 interfaces.
     */
    ~BatteryLifeNode(){};

private:
    /**
     * @brief Sends a recharge request to the robot_location_manager node.
     *
     * This function constructs a `MoveTo` service request directing the robot
     * to its designated charging station. It uses an asynchronous service call.
     */
    void send_recharge_request();

    /** @brief Client used to communicate with the MoveTo service. */
    rclcpp::Client<MoveTo>::SharedPtr client_;

    /**
     * @brief Callback function for the battery state subscriber.
     *
     * @param msg Shared pointer to the received `sensor_msgs::msg::BatteryState` message.
     *
     * This callback updates the current battery percentage and determines
     * whether the robot should initiate a recharge sequence.
     */
    void battery_status_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

    /** 
     * @brief Subscriber to the `/battery_state` topic.
     * 
     * Receives live updates about the robot’s current battery status.
     */
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;

    /**
     * @brief Stores the latest battery percentage value.
     * 
     * Represented as a shared pointer for safe concurrent access if needed.
     */
    std::shared_ptr<float> current_battery_percentage = std::make_shared<float>(0.0f);



    void get_battery( const GetBattery::Request::SharedPtr request,
        const GetBattery::Response::SharedPtr response);

    rclcpp::Service<GetBattery>::SharedPtr battery_server;

};

#endif  // BATTERY_LIFE_NODE_HPP_
