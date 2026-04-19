#ifndef TURTLEBOT3_GAZEBO__ROBOT_POSITION_MANAGER_CLIENT_HPP__
#define TURTLEBOT3_GAZEBO__ROBOT_POSITION_MANAGER_CLIENT_HPP__

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_position_manager_client.hpp
// Author(s): Yunkan Luo
//
// Declaration of the RobotPositionManagerClient singleton class which wraps the
// BatchNavigate action client. This is the renamed replacement for BatchNavigator.


#include <functional>
#include <future>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// Include generated action interface for BatchNavigate
#include "turtlebot3_gazebo/action/batch_navigate.hpp"

/**
 * @brief A robot position management client that can send multiple waypoints
 *        to a navigation action server and receive feedback/progress.
 * 
 * This class implements a singleton pattern and defines a minimal two-state model:
 *  - Available : Ready to accept a new goal
 *  - Busy      : Currently executing or server not ready
 */
class RobotPositionManagerClient {
  public:
    using BatchNavigate = turtlebot3_gazebo::action::BatchNavigate;
    using GoalHandle = rclcpp_action::ClientGoalHandle<BatchNavigate>;

    /** 
     * @brief Summary structure for the navigation result 
     */ 
    struct ResultSummary {
      bool success{};          /// Whether the batch navigation succeeded
      int32_t completed{};     /// Number of waypoints completed
      std::string message;     /// Optional text message from the server
    };

    /** 
     * @brief Simple binary server state
     */
    enum class ServerStatus { Available, Busy };

    /** 
     * @brief Type for a callback function used to notify external users of server state changes 
     */
    using StatusCallback = std::function<void(ServerStatus)>;

    /** 
     * @brief Get the singleton instance, creating it if needed 
     * @return Reference to the singleton instance
     */
    static RobotPositionManagerClient& getInstance() {
      static RobotPositionManagerClient instance;
      return instance;
    }

    /**
     * @brief Delete copy constructor and assignment to ensure singleton
     * @return Reference to the singleton instance
     */
    RobotPositionManagerClient(const RobotPositionManagerClient&) = delete;
    RobotPositionManagerClient& operator=(const RobotPositionManagerClient&) = delete;

    /**
     * @brief Initialize the navigator with a ROS node and action server name
     * 
     * @param node The ROS node to use for communication
     * @param action_name Name of the batch navigate action server
     */
    void initialize(const rclcpp::Node::SharedPtr& node,
                  const std::string& action_name = "batch_navigate");
    
    /**
     * @brief Send a batch of waypoints to navigate to.
     * 
     * @param waypoints The list of target poses to navigate to
     * @param on_feedback Optional callback to report progress (current, total)
     * @return future summarizing the navigation result
     */
    std::shared_future<ResultSummary> navigate(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        std::function<void(int current, int total)> on_feedback = nullptr);
    
    /**
     * @brief Register a callback to be triggered whenever server status changes.
     * @param cb The callback function.
     */
    void set_status_callback(StatusCallback cb);
    /** 
     * @brief Force cancel any active goal. 
     */
    void cancel_all();
    /**
     * @brief Get the current status of the action server.
     * @return The current server status.
     */
    ServerStatus get_status() const;

  private:
    /**
     * @brief Private constructor for singleton
     */
    RobotPositionManagerClient() = default;
    /** 
     * @brief Emit a status change to registered callbacks 
     * 
     * @param st The new server status to emit
     */
    void emit_status(ServerStatus st);
    
    rclcpp::Node::SharedPtr node_; /// ROS node handle
    rclcpp_action::Client<BatchNavigate>::SharedPtr client_; /// Action client

    mutable std::mutex status_mutex_; /// Mutex for status access
    StatusCallback status_cb_{nullptr}; /// Registered status callback
    ServerStatus current_status_{ServerStatus::Available}; /// Cached state

    std::mutex goal_mutex_; /// Mutex for goal handle access
    rclcpp_action::ClientGoalHandle<BatchNavigate>::SharedPtr last_goal_handle_; /// Last sent goal handle

    bool initialized_{false}; /// Flag to track if initialize() was called
};

#endif  // TURTLEBOT3_GAZEBO__ROBOT_POSITION_MANAGER_CLIENT_HPP__
