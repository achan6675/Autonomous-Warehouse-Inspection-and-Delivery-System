#ifndef ROBOT_POSITION_MANAGER_SERVER_HPP_
#define ROBOT_POSITION_MANAGER_SERVER_HPP_

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_position_manager_server.hpp
// Declares RobotPositionManagerServer: a BatchNavigate action server that
// forwards individual NavigateToPose goals to the Nav2 NavigateToPose action.

#include <array>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <vector>

// Include generated BatchNavigate action type and the navigator helper
#include "turtlebot3_gazebo/action/batch_navigate.hpp"
#include "turtlebot3_gazebo/batch_navigator.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

using BatchNavigate = turtlebot3_gazebo::action::BatchNavigate;
using BatchServerHandle = rclcpp_action::ServerGoalHandle<BatchNavigate>;

class RobotPositionManagerServer : public rclcpp::Node {
 public:
  RobotPositionManagerServer();

 private:
  /**
   * @brief Send the next goal in the batch to the NavigateToPose action server
   */
  void sendNextGoal();
  /** 
   * @brief Callback for when a NavigateToPose goal completes
   */
  void goalResultCallback(const NavigateToPoseGoalHandle::WrappedResult& result);
  /** 
   * @brief Handle incoming BatchNavigate goal requests
   */
  void publish_feedback();
  /** 
   * @brief Handle incoming BatchNavigate goal requests
   * 
   * @param uuid - Unique ID for the goal
   * @param goal - The goal message
   * @return GoalResponse indicating acceptance or rejection
   */
  rclcpp_action::GoalResponse handle_goal(
          const rclcpp_action::GoalUUID& uuid,
          std::shared_ptr<const BatchNavigate::Goal> goal);
  /** 
   * @brief Handle incoming BatchNavigate cancel requests
   * 
   * @param goal_handle - The goal handle to cancel
   * @return CancelResponse indicating acceptance or rejection
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<BatchServerHandle> goal_handle);
  /** 
   * @brief Handle accepted BatchNavigate goals
   * 
   * @param goal_handle - The accepted goal handle
   */
  void handle_accepted(const std::shared_ptr<BatchServerHandle> goal_handle);
  
  rclcpp_action::Client<NavigateToPose>::SharedPtr client; /// NavigateToPose action client
  rclcpp_action::Server<BatchNavigate>::SharedPtr batch_server_; /// BatchNavigate action server
  std::vector<std::array<double, 3>> coords; /// Waypoint coordinates

  size_t current_goal_index_; /// Index of the current goal being processed
  bool goal_completed_; /// Flag indicating if the current goal has been completed
  bool goal_active_; /// Flag indicating if the current goal is active
  bool batch_done_; /// Flag indicating if the batch processing is done
  int last_goal_index_; /// Index of the last goal in the batch

  std::shared_ptr<BatchServerHandle> current_batch_goal_handle_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
};

#endif  // ROBOT_POSITION_MANAGER_SERVER_HPP_
