/*
  MTRX3760 2025 Project 2: Warehouse Robot DevKit
  File: damage_manager_node.cpp
  Author(s): Junhao FU

  Implementation of the DamageManagerNode class.
*/

#include "turtlebot3_gazebo/damage_manager_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace turtlebot3_gazebo {

// Constructor initializes the damage manager node, sets up all subscriptions,
// services, and starts the internal state machine timer.
DamageManagerNode::DamageManagerNode() : rclcpp::Node("damage_manager_node") {
  mgr_ = std::make_shared<DamagePointManager>();
  mgr_->start();

  tick_timer_ = this->create_wall_timer(100ms, [this]() { mgr_->tick(); });

  barcode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "barcode", 10,
      std::bind(&DamageManagerNode::barcodeCallback, this,
                std::placeholders::_1));

  qr_xy_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "qr_xy_in_map", 10,
      std::bind(&DamageManagerNode::qrXYCallback, this, std::placeholders::_1));

  using Trigger = std_srvs::srv::Trigger;
  srv_begin_scan_ = this->create_service<Trigger>(
      "test/begin_scan",
      std::bind(&DamageManagerNode::beginScanService, this,
                std::placeholders::_1, std::placeholders::_2));

  srv_mark_done_ = this->create_service<Trigger>(
      "test/mark_scan_done",
      std::bind(&DamageManagerNode::markDoneService, this,
                std::placeholders::_1, std::placeholders::_2));

  srv_dump_ = this->create_service<Trigger>(
      "test/dump", std::bind(&DamageManagerNode::dumpService, this,
                             std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(),
              "damage_manager_node ready. Waiting for /qr_xy_in_map...");
}

// Processes incoming barcode messages and filters out duplicates. When a new
// barcode is detected, it enables acceptance of the next QR position.
void DamageManagerNode::barcodeCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  const std::string& code = msg->data;
  if (code != last_barcode_) {
    last_barcode_ = code;
    accept_next_xy_ = true;
    RCLCPP_INFO(this->get_logger(), "[TEST] New barcode: %s", code.c_str());
  } else {
    RCLCPP_DEBUG(this->get_logger(), "[TEST] Duplicate barcode: %s (ignored)",
                 code.c_str());
  }
}

// Handles QR code position messages from the map frame. Only forwards positions
// to the manager when a new barcode has been detected to prevent duplicates.
void DamageManagerNode::qrXYCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (accept_next_xy_) {
    mgr_->AddObjectFromQR(*msg);
    accept_next_xy_ = false;
    RCLCPP_INFO(this->get_logger(),
                "[TEST] Accepted QR XY for barcode '%s': x=%.3f y=%.3f",
                last_barcode_.empty() ? "<unknown>" : last_barcode_.c_str(),
                msg->point.x, msg->point.y);
  } else {
    RCLCPP_DEBUG(this->get_logger(),
                 "[TEST] Suppressed duplicate QR XY: x=%.3f y=%.3f",
                 msg->point.x, msg->point.y);
  }
}

// Service callback to initiate the scanning phase. Transitions the damage
// point manager to the Scanning state.
void DamageManagerNode::beginScanService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  mgr_->begin_scan();
  res->success = true;
  res->message = "begin_scan triggered";
}

// Service callback to mark scanning as complete. Transitions the damage point
// manager to the SendGoal state, ready for navigation.
void DamageManagerNode::markDoneService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  mgr_->mark_scan_done();
  res->success = true;
  res->message = "mark_scan_done triggered";
}

// Service callback for debugging. Returns a summary string containing the
// current state and object counts from the damage point manager.
void DamageManagerNode::dumpService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  res->success = true;
  res->message = mgr_->dump_summary();
}

}  // namespace turtlebot3_gazebo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turtlebot3_gazebo::DamageManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
