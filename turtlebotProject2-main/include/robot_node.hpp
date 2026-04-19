#ifndef TURTLEBOT3_GAZEBO__ROBOT_NODE_HPP__
#define TURTLEBOT3_GAZEBO__ROBOT_NODE_HPP__

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_node.hpp
// Author(s): Yunkan Luo
//
// Declaration for the RobotNode which integrates behaviours, navigation
// and exposes service endpoints for external clients.


#pragma once
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "turtlebot3_gazebo/robot_position_manager_client.hpp"
#include "turtlebot3_gazebo/delivery_robot.hpp"
#include "turtlebot3_gazebo/IBaseRobot.hpp"
#include "turtlebot3_gazebo/srv/move_to.hpp"
#include "turtlebot3_gazebo/srv/get_text_file.hpp"
#include "turtlebot3_gazebo/srv/get_battery.hpp"
#include "turtlebot3_gazebo/msg/item_entry.hpp"

using MoveTo = turtlebot3_gazebo::srv::MoveTo;
using GetTextFile = turtlebot3_gazebo::srv::GetTextFile;
using ItemEntryMsg = turtlebot3_gazebo::msg::ItemEntry;

/**
 * @brief RobotNode integrates robot behaviors, navigation clients,
 *        and exposes services for external interaction.
 */
class RobotNode : public rclcpp::Node
{
public:
  RobotNode();

  /**
  * @brief Bind the RobotPositionManagerClient to the behavior actions of NavigateManagerDelivery.
  * @param bn RobotPositionManagerClient instance to bind
   */
  void wire_navigator(RobotPositionManagerClient &bn);

private:

  std::shared_ptr<IBaseRobot> delivery_; /// DeliveryRobot behavior instance
  rclcpp::Service<MoveTo>::SharedPtr move_to_srv_; /// /move_to service
  rclcpp::Service<GetTextFile>::SharedPtr text_file_srv_; /// /get_text_file service
  rclcpp::Client<turtlebot3_gazebo::srv::GetBattery>::SharedPtr battery_client_; /// /get_battery client
  rclcpp::TimerBase::SharedPtr battery_timer_; /// Timer for battery status updates
  /// Periodic tick timer to drive the behavior state machine
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

#endif  // TURTLEBOT3_GAZEBO__ROBOT_NODE_HPP__