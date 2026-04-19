/*
  MTRX3760 2025 Project 2: Warehouse Robot DevKit
  File: sim_battery.cpp
  Author(s): Junhao FU

  Implementation of the BatteryStateServerNode class for battery simulation.
*/

#include "turtlebot3_gazebo/sim_battery.hpp"

#include <cmath>

// Constructor
BatteryStateServerNode::BatteryStateServerNode()
    : Node("battery_state_server_node"),
      last_x_(std::numeric_limits<double>::quiet_NaN()),
      last_y_(std::numeric_limits<double>::quiet_NaN()),
      traveled_m_(0.0),
      battery_percent_(100.0) {
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
  drain_per_meter_percent_ =
      this->declare_parameter<double>("drain_per_meter_percent", 5.0);
  low_threshold_percent_ =
      this->declare_parameter<double>("low_threshold_percent", 20.0);
  nominal_voltage_ = this->declare_parameter<double>("nominal_voltage", 12.0);
  idle_drain_percent_per_second_ =
      this->declare_parameter<double>("idle_drain_percent_per_second", 0.001);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(50),
      std::bind(&BatteryStateServerNode::odomCallback, this,
                std::placeholders::_1));

  percent_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("/battery/percentage", 10);
  state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery/state", 10);
  low_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/battery/low", rclcpp::QoS(1).transient_local().reliable());

  get_state_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/battery/get_state",
      std::bind(&BatteryStateServerNode::onGetState, this,
                std::placeholders::_1, std::placeholders::_2));

  battery_ctrl_srv_ =
      this->create_service<turtlebot3_gazebo::srv::BatteryControl>(
          "/battery/control",
          std::bind(&BatteryStateServerNode::onBatteryControl, this,
                    std::placeholders::_1, std::placeholders::_2));

  using namespace std::chrono_literals;
  idle_drain_timer_ = this->create_wall_timer(
      1s, std::bind(&BatteryStateServerNode::onIdleDrain, this));

  RCLCPP_INFO(
      this->get_logger(),
      "Battery simulation started: %.1f%% (drain=%.1f%%/m, idle=%.4f%%/s, "
      "low_threshold=%.1f%%)",
      battery_percent_, drain_per_meter_percent_,
      idle_drain_percent_per_second_, low_threshold_percent_);
}

// Processes odometry updates to track robot movement. Calculates distance
// traveled and drains battery proportionally based on drain_per_meter_percent_
// parameter.
void BatteryStateServerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;

  if (std::isnan(last_x_) || std::isnan(last_y_)) {
    last_x_ = x;
    last_y_ = y;
    return;
  }

  const double dx = x - last_x_;
  const double dy = y - last_y_;
  const double ds = std::hypot(dx, dy);
  if (ds <= 1e-9) {
    return;
  }

  traveled_m_ += ds;
  const double drop = drain_per_meter_percent_ * ds;
  battery_percent_ = std::clamp(battery_percent_ - drop, 0.0, 100.0);

  RCLCPP_INFO(this->get_logger(),
              "Battery: %.1f%% | Traveled: %.2fm | Drop: -%.2f%% | Low: %s",
              battery_percent_, traveled_m_, drop,
              (battery_percent_ <= low_threshold_percent_) ? "YES" : "NO");

  last_x_ = x;
  last_y_ = y;
}

// Service callback that publishes current battery state including percentage,
// voltage, and low battery status. Returns detailed battery information in
// response.
void BatteryStateServerNode::onGetState(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
  const bool is_low = (battery_percent_ <= low_threshold_percent_);

  std_msgs::msg::Float32 pct_msg;
  pct_msg.data = static_cast<float>(battery_percent_);
  percent_pub_->publish(pct_msg);

  sensor_msgs::msg::BatteryState st;
  st.header.stamp = this->get_clock()->now();
  st.voltage =
      static_cast<float>(nominal_voltage_ * (battery_percent_ / 100.0));
  st.current = std::numeric_limits<float>::quiet_NaN();
  st.charge = std::numeric_limits<float>::quiet_NaN();
  st.capacity = std::numeric_limits<float>::quiet_NaN();
  st.design_capacity = std::numeric_limits<float>::quiet_NaN();
  st.percentage = static_cast<float>(battery_percent_ / 100.0);
  st.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  st.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  st.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  st.present = true;
  state_pub_->publish(st);

  std_msgs::msg::Bool low_msg;
  low_msg.data = is_low;
  low_pub_->publish(low_msg);

  resp->success = true;
  char buf[256];
  std::snprintf(buf, sizeof(buf), "battery=%.1f%%, traveled=%.2f m, low=%s",
                battery_percent_, traveled_m_, is_low ? "true" : "false");
  resp->message = buf;
}

// Service callback for battery control requests. Returns current battery
// percentage and low battery status without modifying state.
void BatteryStateServerNode::onBatteryControl(
    const std::shared_ptr<turtlebot3_gazebo::srv::BatteryControl::Request> req,
    std::shared_ptr<turtlebot3_gazebo::srv::BatteryControl::Response> resp) {
  resp->percentage = static_cast<float>(battery_percent_);
  resp->is_low = (battery_percent_ <= low_threshold_percent_);

  RCLCPP_INFO(this->get_logger(),
              "Service called: Battery=%.1f%%, Traveled=%.2fm, Low=%s",
              battery_percent_, traveled_m_, resp->is_low ? "YES" : "NO");
}

// Timer callback for idle battery drain. Drains battery at
// idle_drain_percent_per_second_ rate when robot is stationary, simulating
// standby power consumption.
void BatteryStateServerNode::onIdleDrain() {
  if (battery_percent_ <= 0.0) {
    return;
  }

  battery_percent_ =
      std::clamp(battery_percent_ - idle_drain_percent_per_second_, 0.0, 100.0);

  RCLCPP_DEBUG(this->get_logger(), "Idle drain: Battery now at %.3f%%",
               battery_percent_);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryStateServerNode>());
  rclcpp::shutdown();
  return 0;
}
