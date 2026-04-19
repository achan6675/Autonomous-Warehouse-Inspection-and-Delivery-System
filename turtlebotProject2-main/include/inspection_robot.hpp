/*
  MTRX3760 2025 Project 2: Warehouse Robot DevKit
  File: inspection_robot.hpp
  Author(s): Junhao FU

  This file defines the InspectionRobot class, which manages the detection
  and tracking of damaged points in a warehouse environment. This node is
  exposed to a interface which is able to recieved a flag of finishing manual
  scanning and save the list of damage point position information in vector,
  which act as a service server. This class inherities to IBhaviour to receice
  the requeset of asking position information for navigator. The manager handles
  the scanning process, converts QR code positions to grid entries, and
  maintains state through a finite state machine. The process of the state
  machine is Idle - Scanning - SendGoal - Processing - Finished.
*/

#ifndef TURTLEBOT3_GAZEBO__INSPECTION_ROBOT_HPP_
#define TURTLEBOT3_GAZEBO__INSPECTION_ROBOT_HPP_

#include <cstdint>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "turtlebot3_gazebo/IBehavior.hpp"
#include "turtlebot3_gazebo/msg/item_entry.hpp"

namespace turtlebot3_gazebo {

using ItemEntry = turtlebot3_gazebo::msg::ItemEntry;

class InspectionRobot final : public IBehavior {
 public:
  enum class ScanState { Idle, Scanning, SendGoal, Processing, Finished };

  // Constructor and destructor
  InspectionRobot();
  ~InspectionRobot() override;

  // Resets all internal state, clear the damaged point list
  void start() override;
  // Cancels the current operation
  void cancel() override;
  // Get current behavior status
  BehaviorStatus GetStatus() const override;
  // Update robot state
  void onState(const RobotState& s) override;

  // Begin scanning for damaged points
  void begin_scan();
  // Mark scanning as done
  void mark_scan_done();
  // Send goal to navigate to damaged points
  void send_goal();
  // Bind ROS2 services
  void bind_services(const rclcpp::Node::SharedPtr& node);

  // Add object from ItemEntry
  void AddObject(ItemEntry item) override;
  // Add object from QR code position
  void AddObjectFromQR(geometry_msgs::msg::PointStamped qr_position);
  // Ticking function to update state machine
  void tick() override;

  // Return damage points as ItemEntry list
  std::vector<ItemEntry> GetObjects(ObjectOption option) const override;
  // Reeturn damage points as ObjectInfo format with position information and
  // metadata, filtered by the same options as GetObjects()
  std::vector<ObjectInfo> GetObjectsAsInfo(ObjectOption option) const;
  // Returns direct const reference to the internal ItemEntry storage for
  // efficient access without copying
  const std::vector<turtlebot3_gazebo::msg::ItemEntry>& get_item_entries()
      const;

  // Return current position in the state machine
  ScanState scan_state() const;

  // Required by IBehavior interface but not used in this behaviour
  // implmentation
  void set_return_pose(const geometry_msgs::msg::Pose& p) override;

  // Generates a debug string with state name and object counts in the format
  // "state=X, damaged_count=Y, item_entries=Z"
  std::string dump_summary() const;

 private:
  // Map length in meters along x axis
  static constexpr double MAP_LENGTH_M = 2.36;
  // Number of grid rows for binning damaged points
  static constexpr int NUM_ROWS = 6;
  // Fxied column for all damaged points
  static constexpr int FIXED_COLUMN = 1;

  // Mutex for thread-safe concurrent access
  mutable std::mutex mu_;
  // Current behavior status (active, phase, progress, note)
  BehaviorStatus status_;
  // Auto-incrementing ID for damage points
  unsigned int next_id_{1};

  // Scan state (Idle/Scanning/SendGoal/Processing/Finished)
  ScanState scan_state_{ScanState::Idle};

  // Internal list of damage points with pose information
  std::vector<ObjectInfo> damaged_list_;
  // Grid-based representation (row, column) for navigation
  std::vector<turtlebot3_gazebo::msg::ItemEntry> item_entries_;

  // Service to start scanning
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_begin_scan_;
  // Service to mark scanning as complete
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_mark_done_;
  // Service to initiate navigation
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_send_goal_;
};

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__inspection_robot_HPP_
