/*
  MTRX3760 2025 Project 2: Warehouse Robot DevKit
  File: qr_transform.hpp
  Author(s): Junhao FU

  This file defines the TFStaticBroadcaster class for managing coordinate
  transformations in the warehouse robot system. It handles static transforms
  (base_link to camera_link) and dynamic transforms (QR code detections from
  camera frame to map frame). The node subscribes to QR pose detections and
  publishes the transformed positions in the map coordinate system.
*/

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TFStaticBroadcaster : public rclcpp::Node {
 public:
  // Constructor for TF Static Broadcaster Node
  TFStaticBroadcaster();

 private:
  // Broadcast static transform that defines the fixed spatial relationship
  // between the robot base and the camera frame. This transform is static and
  // only needs to be published once at startup.
  void publish_static_base_to_camera();
  // Receiving QR code pose detections in camera frame, tansforms them to map
  // to map frame using TF trees, boardcasts dynamic transform for the QR cdoe
  // and publishes both the full pose and XY position in map coordinates.
  void qrPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Static transform boadcaster for base_link to camera_link
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  // Dynamic transform broadcaster for QR code detections
  std::shared_ptr<tf2_ros::TransformBroadcaster> dyn_broadcaster_;
  // TF buffer for storing and querying transforms
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // TF listener to populate the TF buffer
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscription to QR code pose detections
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr qr_sub_;

  // Publisher for transformed QR code poses in map frame
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr qr_in_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr qr_xy_pub_;

  // Frame ID of the camera
  std::string camera_frame_id_;

  // Extra yaw adjustment in degrees for QR code orientation
  double extra_yaw_deg_ = 0.0;
};
