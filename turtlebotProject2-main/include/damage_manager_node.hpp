/*
  MTRX3760 2025 Project 2: Warehouse Robot DevKit
  File: damage_manager_node.hpp
  Author(s): Junhao FU

  This header defines the DamageManagerNode class, which acts as a ROS 2 node
  wrapper around the DamagePointManager. It handles subscription to QR code
  detection topics, manages barcode deduplication, and provides service
  interfaces for controlling the damage point scanning workflow.
*/

#ifndef TURTLEBOT3_GAZEBO__DAMAGE_MANAGER_NODE_HPP_
#define TURTLEBOT3_GAZEBO__DAMAGE_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "turtlebot3_gazebo/inspection_robot.hpp"

namespace turtlebot3_gazebo {

/**
 * @class DamageManagerNode
 * @brief ROS 2 node that manages damage point detection and tracking workflow.
 *
 * This node subscribes to barcode and QR position topics, filters duplicate
 * detections, and forwards valid damage points to the DamagePointManager.
 * It provides service interfaces for scan control and debugging.
 */
class DamageManagerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor - initializes the node and sets up all subscriptions
   * and services.
   */
  DamageManagerNode();

  /**
   * @brief Destructor
   */
  ~DamageManagerNode() override = default;

 private:
  /**
   * @brief Callback for barcode topic - detects new barcodes and gates
   * duplicate XY coordinates.
   * @param msg Barcode string message
   */
  void barcodeCallback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Callback for QR position topic - forwards valid positions to the
   * manager.
   * @param msg QR code position in map frame
   */
  void qrXYCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  /**
   * @brief Service callback to begin scanning for damage points.
   * @param req Service request (unused)
   * @param res Service response with success status
   */
  void beginScanService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief Service callback to mark scanning as complete.
   * @param req Service request (unused)
   * @param res Service response with success status
   */
  void markDoneService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /**
   * @brief Service callback to dump current manager state for debugging.
   * @param req Service request (unused)
   * @param res Service response with state summary string
   */
  void dumpService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Core manager instance
  std::shared_ptr<InspectionRobot> mgr_;

  // ROS 2 communication
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr barcode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr qr_xy_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  // Service servers
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_begin_scan_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_mark_done_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_dump_;

  // Deduplication state
  std::string last_barcode_;
  bool accept_next_xy_{true};
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__DAMAGE_MANAGER_NODE_HPP_
