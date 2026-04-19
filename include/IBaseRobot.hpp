#ifndef TURTLEBOT3_GAZEBO__IBASEROBOT_HPP__
#define TURTLEBOT3_GAZEBO__IBASEROBOT_HPP__

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: IBaseRobot.hpp
// Author(s): Yunkan Luo
//
// Interface for pluggable robot behaviours. Defines lifecycle methods,
// order representation and the Act structure used to inject navigation actions.


#include <functional>
#include <string>
#include <vector>
#include <iostream>

#include <geometry_msgs/msg/pose.hpp>
#include "turtlebot3_gazebo/msg/item_entry.hpp"

// 这里直接包含是为了使用 RobotPositionManagerClient::ResultSummary / ServerStatus
#include "turtlebot3_gazebo/robot_position_manager_client.hpp"

using ItemEntry = turtlebot3_gazebo::msg::ItemEntry;

// extendable robot state (e.g. battery level can be added later)
struct RobotState {};

// Simple structure to report robot status
struct RobotStatus {
  bool         active{false};
  std::string  phase;            // e.g. "idle", "navigating", ...
  int          progress_current{0};
  int          progress_total{0};
  std::string  note;             // log / error / info
};

/**
 * @brief Interface for pluggable robot behaviours.
 * 
 * Defines lifecycle methods, order representation and the Act structure
 * used to inject navigation actions.
 */
class IBaseRobot {
public:
    // unified order representation (implementations can freely convert/extend)
  struct ObjectInfo {
    geometry_msgs::msg::Pose pose;
    std::string              type;
    unsigned int             item_id{0};
  };
  // Options for querying objects
  enum ObjectOption {
    PENDING_OBJECTS   = 1,
    COMPLETED_OBJECTS = 2,
    PROCESSING_OBJECTS= 3,
  };
  virtual ~IBaseRobot() = default;

  // lifecycle
  /** @brief Start the robot and initialize necessary components */
  virtual void start() = 0;
  /** @brief Handle robot state updates (not used in current implementations) */
  virtual void onState(const RobotState& s) = 0;
  /** @brief Cancel current actions */
  virtual void cancel() = 0;
  /** @brief Get current behavior status */
  virtual RobotStatus GetStatus() const = 0;

  /** 
  * @brief Add an object to the robot's perception
  * @param item - the item to add
  */
  virtual void AddObject(ItemEntry item) = 0;
  /** @brief Update the robot's internal state */
  virtual void tick() = 0;

  /**
   * @brief Return a list of objects (converted to ItemInfo/ItemEntry) by option
   * @param option - the type of objects to retrieve
   * @return vector of ItemEntry objects matching the specified option
   */
  virtual std::vector<ItemEntry> GetObjects(ObjectOption option) const = 0;

  // Actions that the robot can perform, to be bound externally
  struct Act {
    /** 
    * @brief Send a batch of paths; can provide feedback (current,total), returns a summary future
    * @param poses - the target poses to navigate to
    * @param on_feedback - callback to report progress
    * @return future summarizing the navigation result
    */
    std::function<std::shared_future<RobotPositionManagerClient::ResultSummary>(
      const std::vector<geometry_msgs::msg::Pose>&,
      std::function<void(int,int)> on_feedback)> navigate;

    /** @brief Cancel all in-flight goals */
    std::function<void()> cancel_all;

    /** @brief Query navigation server status (Available / Busy / …) */
    std::function<RobotPositionManagerClient::ServerStatus()> query_server;
  };

  /** 
   * @brief Allow external binding of implemented Act
   * 
   * @param a - the Act structure to bind
   */
  void bind_actions(Act &a) { act_ = a; }
  /** 
   * @brief Set the pose to navigate to when the behavior reaches COMPLETED. 
   * 
   * @param p - the target pose
   */
  /** @brief Set the pose to return to after completing the behavior */
  virtual void set_return_pose(const geometry_msgs::msg::Pose &p) = 0;
protected:
  explicit IBaseRobot(const std::string& name = "IBaseRobot") {
    std::cout << "[Behavior] " << name << " instantiated." << std::endl;
  }

  Act act_;  /// DeliveryRobot calls navigate/cancel_all/query_server through this
};

#endif  // TURTLEBOT3_GAZEBO__IBASEROBOT_HPP__
