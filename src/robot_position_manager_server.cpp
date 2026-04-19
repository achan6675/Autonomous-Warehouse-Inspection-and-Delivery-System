#include "turtlebot3_gazebo/robot_position_manager_server.hpp"

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_position_manager_server.cpp
// Implements RobotPositionManagerServer methods and main().
#include "turtlebot3_gazebo/robot_position_manager_server.hpp"

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_position_manager_server.cpp
// Implements RobotPositionManagerServer methods and main().

#include <cmath>

// ---------------------- RobotPositionManagerServer Implementation ----------------------
RobotPositionManagerServer::RobotPositionManagerServer() : Node("robot_position_manager_server") {
  RCLCPP_INFO(get_logger(), "RobotPositionManagerServer started");
  client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  nav_client_ = client;

  batch_server_ = rclcpp_action::create_server<BatchNavigate>(
      this, "batch_navigate",
      std::bind(&RobotPositionManagerServer::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&RobotPositionManagerServer::handle_cancel, this,
                std::placeholders::_1),
      std::bind(&RobotPositionManagerServer::handle_accepted, this, std::placeholders::_1));

  // initialize state
  current_goal_index_ = 0;
  goal_completed_ = false;
  last_goal_index_ = -1;
  goal_active_ = false;
  batch_done_ = false;
}

// Send the next goal in the batch to the NavigateToPose action server
void RobotPositionManagerServer::sendNextGoal() {
  if (current_goal_index_ >= coords.size()) {
    RCLCPP_INFO(get_logger(), "All waypoints completed!");
    batch_done_ = true;
    goal_active_ = false;

    if (current_batch_goal_handle_) {
      auto result = std::make_shared<BatchNavigate::Result>();
      result->success = true;
      result->completed = last_goal_index_;
      result->message = "All waypoints navigated successfully.";
      current_batch_goal_handle_->succeed(result);
      current_batch_goal_handle_.reset();
    }
    return;
  }

  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Action server not available.");
    return;
  }

  const auto& point = coords.at(current_goal_index_);

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = now();

  goal_msg.pose.pose.position.x = point[0];
  goal_msg.pose.pose.position.y = point[1];
  goal_msg.pose.pose.position.z = point[2];

  RCLCPP_INFO(get_logger(), "Goal Coordinate %zu: x=%.2f, y=%.2f, theta=%.2f",
              current_goal_index_, point[0], point[1], point[2]);

  RCLCPP_INFO(get_logger(), "Sending goal...");
  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.result_callback =
      std::bind(&RobotPositionManagerServer::goalResultCallback, this, std::placeholders::_1);
  client->async_send_goal(goal_msg, options);

  goal_active_ = true;
  batch_done_ = false;
  goal_completed_ = false;
}

// Callback for when a NavigateToPose goal completes
rclcpp_action::CancelResponse RobotPositionManagerServer::handle_cancel(
    const std::shared_ptr<BatchServerHandle> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Cancel request received.");
  if (nav_client_) nav_client_->async_cancel_all_goals();
  return rclcpp_action::CancelResponse::ACCEPT;
}

// Callback for when a NavigateToPose goal completes
void RobotPositionManagerServer::goalResultCallback(
    const NavigateToPoseGoalHandle::WrappedResult& result) {
  (void)result;
  RCLCPP_INFO(get_logger(), "Navigation Completed for waypoint %zu",
              current_goal_index_);

  goal_completed_ = true;
  last_goal_index_ = static_cast<int>(current_goal_index_);

  publish_feedback();

  current_goal_index_++;
  goal_active_ = false;

  sendNextGoal();
}

// Handle incoming BatchNavigate goal requests
rclcpp_action::GoalResponse RobotPositionManagerServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const BatchNavigate::Goal> goal) {
  (void)uuid;
  if (goal->waypoints.size() == 0 || goal->waypoints.size() > 10) {
    RCLCPP_WARN(get_logger(),
                "Reject goal: waypoints size must be 1..10 (got %zu).",
                goal->waypoints.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  bool nav_ready =
      nav_client_->wait_for_action_server(std::chrono::milliseconds(1));
  if (!nav_ready) {
    RCLCPP_WARN(get_logger(), "Reject goal: Nav2 action not ready.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Received batch navigation goal request");

  goal_active_ = true;
  batch_done_ = false;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Handle incoming BatchNavigate goal requests
void RobotPositionManagerServer::handle_accepted(
    const std::shared_ptr<BatchServerHandle> goal_handle) {
  current_batch_goal_handle_ = goal_handle;

  auto goal = goal_handle->get_goal();
  coords.clear();
  coords.reserve(goal->waypoints.size());
  for (const auto& wp : goal->waypoints) {
    coords.push_back({wp.position.x, wp.position.y, wp.orientation.z});
  }

  current_goal_index_ = 0;
  last_goal_index_ = -1;
  goal_completed_ = false;
  goal_active_ = false;
  batch_done_ = false;

  sendNextGoal();
}

// Handle incoming BatchNavigate goal requests
void RobotPositionManagerServer::publish_feedback() {
  if (!current_batch_goal_handle_) return;
  auto fb = std::make_shared<BatchNavigate::Feedback>();
  fb->current_index = last_goal_index_ + 1;
  fb->total = static_cast<int32_t>(coords.size());
  current_batch_goal_handle_->publish_feedback(fb);
  RCLCPP_INFO(this->get_logger(),
              "Published feedback: %d / %d", fb->current_index, fb->total);
}

// ---------------------- Main Function ----------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotPositionManagerServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
