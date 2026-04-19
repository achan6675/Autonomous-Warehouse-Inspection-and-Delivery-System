#ifndef TURTLEBOT3_GAZEBO__DELIVERY_ROBOT_HPP__
#define TURTLEBOT3_GAZEBO__DELIVERY_ROBOT_HPP__

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: delivery_robot.hpp
// Author(s): Yunkan Luo
//
// Declaration of the DeliveryRobot behaviour class used to manage
// delivery orders and interact with the RobotPositionManagerClient client.


#include "turtlebot3_gazebo/IBaseRobot.hpp"
#include "turtlebot3_gazebo/robot_position_manager_client.hpp"
#include <algorithm>
#include <cmath>
#include <future>

using ItemEntry = turtlebot3_gazebo::msg::ItemEntry;

/**
 * @brief Manages the delivery of orders to specified locations using a TurtleBot3
 *
 * This class implements a state machine that handles:
 * - Receiving and queueing delivery orders
 * - Sorting orders for optimal delivery route
 * - Managing the delivery process using RobotPositionManagerClient
 * - Tracking order status (pending, processing, completed)
 */
class DeliveryRobot : public IBaseRobot {
  public:
    typedef enum {
      IDLE,
      SENDING_GOALS,
      IN_PROGRESS,
      COMPLETED
    } State;

  // Reuse the ObjectInfo and ObjectOption from IBaseRobot for a stable interface
  using ObjectInfo = IBaseRobot::ObjectInfo;
  using ObjectOption = IBaseRobot::ObjectOption;
  
  /**
   * @brief Constructor for DeliveryRobot
   */
  DeliveryRobot();
  /**
   * @brief Start the delivery process
   */
  void start() override;
  /**
   * @brief Cancel the delivery process
   */
  void cancel() override;
  /**
   * @brief Get current behavior status
   * @return BehaviorStatus structure with current status
   */
  BehaviorStatus GetStatus() const override;
  /**
   * @brief Add a delivery order
   * @param item The item entry representing the order
   */
  void AddObject(ItemEntry item) override;
  /**
   * @brief Handle robot state updates (not used in current implementations)
   */
  void onState(const RobotState& s) override {}
  /**
   * @brief Update the robot's behavior state
   * uses the state machine to progress through delivery phases
   */
  void tick() override;
  /**
   * @brief Set the pose to navigate to when the behavior reaches COMPLETED.
   * 
   * @param p The target pose to return to after completing deliveries
   */
  void set_return_pose(const geometry_msgs::msg::Pose &p) override { return_pose_ = p; }
  /**
   * @brief Get a list of objects based on the specified option
   * 
   * @param option The type of objects to retrieve (pending, completed, processing)
   * @return Vector of ItemEntry objects matching the specified option
   */
  std::vector<ItemEntry> GetObjects(ObjectOption option) const override;
  
  private:
    /**
     * @brief Convert state enum to string representation
     */
    std::string phaseName(State s) const;

    /**
     * @brief State handlers for the state machine
     */
    void IDLEHandle();
    /**
     * @brief Handle the SENDING_GOALS state
     */
    void SENDING_GOALSHandle();
    /**
     * @brief Handle the IN_PROGRESS state
     */
    void IN_PROGRESSHandle();
    /**
     * @brief Handle the COMPLETED state
     */
    void COMPLETEDHandle();

    /**
     * @brief Sort orders based on their coordinates for optimal delivery route
     */
  std::vector<ObjectInfo> sortOrders(const std::vector<ObjectInfo>& items) const;
    
    /**
     * @brief Extract poses from order information for navigation
     */
  std::vector<geometry_msgs::msg::Pose> extractPoses(const std::vector<ObjectInfo>& items) const;
    
    // Constants
  static constexpr double COORDINATE_EPSILON = 1e-5;  /// Epsilon for coordinate comparison
    
    // Inner data (protected by mutex)
  mutable std::mutex mx_;
  bool active_{false};              /// Whether the behavior is active
  bool in_flight_{false};           /// Whether there is ongoing navigation
  bool one_item_delivered_{false};  /// Whether a single item has been delivered
  int id_counter_{0};               /// Unique ID counter for orders
  std::vector<ObjectInfo> pending_orders_;    /// Orders waiting to be processed
  std::vector<ObjectInfo> completed_orders_;  /// Successfully delivered orders
  std::vector<ObjectInfo> processing_orders_; /// Orders currently being delivered
  std::pair<int,int> progress_{0,0};        /// (current, total) progress
  std::string note_;                        /// Status message
  State current_state{IDLE};                /// Current state of the state machine
  State next_state{IDLE};                   /// Next state to transition to
  State last_state_{IDLE};                  /// For transition logging

  // Future for ongoing navigation (used for non-blocking check of completion/failure)
  std::shared_future<RobotPositionManagerClient::ResultSummary> future_;
  // Optional return/home pose to navigate to after completion
  geometry_msgs::msg::Pose return_pose_;
  
};

#endif // TURTLEBOT3_GAZEBO__DELIVERY_ROBOT_HPP__
