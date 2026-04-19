#include "turtlebot3_gazebo/robot_position_manager_client.hpp"
#include "turtlebot3_gazebo/action/batch_navigate.hpp"

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_position_manager_client.cpp
// Author(s): Yunkan Luo
//
// Implements the RobotPositionManagerClient singleton which provides a thin client
// wrapper around the BatchNavigate action server for sending batched goals
// and receiving feedback/results.

#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

// Initialize the singleton client
void RobotPositionManagerClient::initialize(const rclcpp::Node::SharedPtr& node,
                                const std::string& action_name) {
  if (initialized_) {
    RCLCPP_WARN(node->get_logger(), "RobotPositionManagerClient already initialized");
    return;
  }

  node_ = node;
  client_ = rclcpp_action::create_client<BatchNavigate>(node_, action_name);
  initialized_ = true;
}

// Set status change callback
void RobotPositionManagerClient::set_status_callback(StatusCallback cb) {
  std::scoped_lock lk(status_mutex_);
  status_cb_ = std::move(cb);
}

// Emit status change to registered callbacks
void RobotPositionManagerClient::emit_status(ServerStatus st) {
  std::scoped_lock lk(status_mutex_);
  current_status_ = st;
  if (status_cb_) status_cb_(st);
}

// Get the current status of the action server.
RobotPositionManagerClient::ServerStatus RobotPositionManagerClient::get_status() const {
  std::scoped_lock lk(status_mutex_);
  return current_status_;
}

// Send a batch of waypoints to navigate to.
std::shared_future<RobotPositionManagerClient::ResultSummary>
RobotPositionManagerClient::navigate(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    std::function<void(int current, int total)> on_feedback)
{
  if (!initialized_) {
    throw std::runtime_error("RobotPositionManagerClient not initialized. Call initialize() first.");
  }

  auto prom = std::make_shared<std::promise<ResultSummary>>();
  auto fut = prom->get_future().share();

  // Mark Busy immediately once a goal attempt starts
  emit_status(ServerStatus::Busy);

  // Wait for action server availability
  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "Action server unavailable");
    prom->set_value({false, 0, "Action server unavailable"});
    return fut;
  }

  // Prepare goal
  BatchNavigate::Goal goal_msg;
  goal_msg.waypoints = waypoints;

  // Define send goal options (response, feedback, result)
  auto options = rclcpp_action::Client<BatchNavigate>::SendGoalOptions();

  // When goal is accepted/rejected by server
  options.goal_response_callback =
      [this, prom](GoalHandle::SharedPtr gh) {
        if (!gh) {
          RCLCPP_ERROR(node_->get_logger(), "Goal rejected by server");
          prom->set_value({false, 0, "Goal rejected"});
          return;
        }
        std::scoped_lock lk(goal_mutex_);
        last_goal_handle_ = gh;
      };
  // Feedback callback: passes progress back to behavior layer
  options.feedback_callback =
      [on_feedback, this](GoalHandle::SharedPtr,
                    const std::shared_ptr<const BatchNavigate::Feedback> fb) {
        if (fb) {
          RCLCPP_INFO(this->node_->get_logger(), "[RobotPositionManagerClient] feedback %d / %d", fb->current_index, fb->total);
          if (on_feedback) on_feedback(fb->current_index, fb->total);
        }
      };

  // Final result callback
  options.result_callback =
      [this, prom](const GoalHandle::WrappedResult& r) {
        ResultSummary summary{};
        switch (r.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            summary.success = true;
            summary.message = "succeeded";
            break;
          case rclcpp_action::ResultCode::ABORTED:
            summary.success = false;
            summary.message = "aborted";
            break;
          case rclcpp_action::ResultCode::CANCELED:
            summary.success = false;
            summary.message = "canceled";
            break;
          default:
            summary.success = false;
            summary.message = "unknown";
            break;
        }

        // Clear last goal and mark as Available
        {
          std::scoped_lock lk(goal_mutex_);
          last_goal_handle_.reset();
        }
        emit_status(ServerStatus::Available);
        prom->set_value(summary);
      };

  try {
    // Send the goal asynchronously
    client_->async_send_goal(goal_msg, options);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "async_send_goal exception: %s", e.what());
    prom->set_value({false, 0, std::string("exception: ") + e.what()});
  }

  return fut;
}

// Force cancel any active goal.
void RobotPositionManagerClient::cancel_all() {
  if (!initialized_) {
    return;
  }

  rclcpp_action::ClientGoalHandle<BatchNavigate>::SharedPtr gh;
  {
    std::scoped_lock lk(goal_mutex_);
    gh = last_goal_handle_;
  }

  // If nothing is running, simply mark Available
  if (!gh) {
    emit_status(ServerStatus::Available);
    return;
  }

  try {
    client_->async_cancel_goal(gh, [this](auto) {
      // Once cancel request acknowledged, mark system Available again
      emit_status(ServerStatus::Available);
    });
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cancel exception: %s", e.what());
  }

  {
    std::scoped_lock lk(goal_mutex_);
    last_goal_handle_.reset();
  }
}
