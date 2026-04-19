/*
  MTRX3760 2025 Project 2: Warehouse Robot DevKit
  File: qr_detector.hpp
  Author(s): Junhao FU

  This file is the header file for the QR code detector node in ROS2.
  It defines the QRDetector class, which subscribes to image and camera info
  topics, processes images to detect QR codes using the ZBar library. Meanwhile,
  it uses solvePNP to determine the 3D pose of the detected QR codes based on
  their corners and the displacements between the camera frame and QR code
  frame. The node then publishes the detected QR code poses and information.
*/
#ifndef QR_DETECTOR_HPP_
#define QR_DETECTOR_HPP_

#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <unordered_map>

#include "zbar.h"
#include "zbar_ros_interfaces/msg/symbol.hpp"

namespace zbar_ros {
class QRDetector : public rclcpp::Node {
 public:
  // Constructor for QR detector node
  // Initializes subscription, publication, and ZBar scanner
  QRDetector();

 private:
  // Callback functions for processing incoming camera images to detect QR codes
  // using ZBar library. WHen the QR codes is detected, solvePnP is applied to
  // obtain 3D pose position of QR codes. Lastly, publishing their poses,
  // orientation and decorded data of QR codes
  void imageCb(sensor_msgs::msg::Image::ConstSharedPtr msg);
  // Callback function to clean up old entries in the barcode memory to prevent
  // duplicate
  // detections within the throttle period. It is called periodically by a
  // timer.
  void cleanCb();
  // A function to receive and store camera calibration parameters intrinsic
  // matrix and distortion coefficients from camera info topic, which are used
  // for PnP pose estimation of detected QR codes.
  void cameraInfoCb(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  // A function to arranges four corners of a detected QR code in clockwise
  // order starting from the top-left corner. It uses centroid calculation and
  // angle based sorting to ensure consistent corner ordering for PnP solver.
  static void sortCornersClockwise(std::vector<cv::Point2f>& pts);
  // A function to approximate a polygon to a quadrilateral with exactly four
  // corners. It uses contour approximation to simplify the polygon shape and
  // extract the four corner points of the quadrilateral.
  static bool fourCornersFromPolygon(const std::vector<cv::Point2f>& poly,
                                     std::vector<cv::Point2f>& out4);

  // Claim for subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_sub_;

  // Claim for publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_deg_pub_;
  rclcpp::Publisher<zbar_ros_interfaces::msg::Symbol>::SharedPtr symbol_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr barcode_pub_;

  // Timer for cleaning up old barcode entries
  rclcpp::TimerBase::SharedPtr clean_timer_;

  // ZBar image scanner for QR code detection
  zbar::ImageScanner scanner_;

  // Camera calibration parameters
  cv::Mat K_;
  cv::Mat dist_;

  // Mutex and memory to track recently detected barcodes to prevent
  // duplicate detections within a throttle period
  std::mutex memory_mutex_;
  std::unordered_map<std::string, rclcpp::Time> barcode_memory_;
  std::string camera_frame_id_;

  // Flag to indicate if camera info has been received
  bool caminfo_ready_{false};
  // Throttle time in seconds to prevent duplicate detections
  double throttle_;
  // Physical size of the QR code in meters
  double qr_size_in_m_;
};
}  // namespace zbar_ros

#endif  // QR_DETECTOR_HPP_