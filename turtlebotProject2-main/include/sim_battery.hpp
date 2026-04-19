/*
  MTRX3760 2025 Project 2: Warehouse Robot DevKit
  File: sim_battery.hpp
  Author(s): Junhao FU

  This file defines the BatteryStateServerNode class for simulating battery
  depletion based on robot odometry. This node is build by considering the
  battery life of turtlbot in real world is too long, which may not be suitable
  for fully demonstrates the whole functionality which related to the battery.
  Hence, this node is created to simulate faster battery depletion. The node
  tracks distance traveled and drains battery percentage accordingly, providing
  services to query battery state and publishing battery information on demand.
*/

#ifndef SIM_BATTERY_HPP_
#define SIM_BATTERY_HPP_

#include <limits>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "turtlebot3_gazebo/srv/battery_control.hpp"

class BatteryStateServerNode : public rclcpp::Node {
 public:
  // Constructor for Battery State Server Node
  BatteryStateServerNode();

 private:
  // Tracks robot movement by calculating distance traveled between consecutive
  // odometry updates, drains battery percentage based on distance, and logs
  // real-time battery status.
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Publkishes current battery state to all battery topics and return
  // a summary message with current status.
  void onGetState(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  // Handle battery control requests, returns current battery percentage and low
  // battery flag.
  void onBatteryControl(
      const std::shared_ptr<turtlebot3_gazebo::srv::BatteryControl::Request>
          req,
      std::shared_ptr<turtlebot3_gazebo::srv::BatteryControl::Response> resp);
  // Drains battery at a slow rate when the robot is in stationary. Called
  // periodically by idle_drain_timer_ to simulate standby power consumption.
  void onIdleDrain();

  // Odometry topic name
  std::string odom_topic_;
  // Battery drain rate (%/meter)
  double drain_per_meter_percent_;
  // Low battery threshold (%)
  double low_threshold_percent_;
  // Nominal battery voltage (V)
  double nominal_voltage_;
  // Idle drain rate (%/second)
  double idle_drain_percent_per_second_;
  // Last X position from odometry (m)
  double last_x_;
  // Last Y position from odometry (m)
  double last_y_;
  // Total distance traveled (m)
  double traveled_m_;
  // Current battery percentage (0-100%)
  double battery_percent_;

  // Subscription to odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publishers for battery information
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr percent_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr low_pub_;

  // Service servers for battery state and control
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_state_srv_;
  rclcpp::Service<turtlebot3_gazebo::srv::BatteryControl>::SharedPtr
      battery_ctrl_srv_;

  // Timer for idle drain
  rclcpp::TimerBase::SharedPtr idle_drain_timer_;
};

#endif  // SIM_BATTERY_HPP_
