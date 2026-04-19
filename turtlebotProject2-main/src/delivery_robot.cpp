// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: delivery_robot.cpp
// Author(s): Yunkan Luo
//
// Implements the DeliveryRobot behaviour which manages queuing,
// sorting and dispatching delivery orders to the RobotPositionManagerClient action client.


#include "turtlebot3_gazebo/delivery_robot.hpp"


// Coordinate mapping constants used by this robot
static constexpr double kXBase = -0.0544;
static constexpr double kXStep = 2.171 / 3.0; // 0.7236666666666667
static constexpr double kYBase = 0.0192;
static constexpr double kYStep = 0.1;

// ---------------------- DeliveryRobot Implementation ----------------------
DeliveryRobot::DeliveryRobot() : IBaseRobot("DeliveryRobot") {
  pending_orders_.clear();
  completed_orders_.clear();
  processing_orders_.clear();
  id_counter_ = 1;
  note_ = "Instantiated";
}

// Initialize any necessary components (if needed)
void DeliveryRobot::start() {
  RCLCPP_INFO(rclcpp::get_logger("behavior"),
              "[Behavior] DeliveryRobot started.");

  active_ = true;
  in_flight_ = false;
  current_state = IDLE;
  next_state = IDLE;
  progress_ = {0, 0};
  note_ = "Started";
}

// Cancel current operations and reset state
void DeliveryRobot::cancel() {
  std::lock_guard<std::mutex> lock(mx_);
  RCLCPP_INFO(rclcpp::get_logger("behavior"),
              "[Behavior] DeliveryRobot cancelled.");

  if (current_state == IN_PROGRESS && in_flight_) {
  act_.cancel_all();

    // Move processing orders back to pending
    pending_orders_.insert(
      pending_orders_.end(),
      std::make_move_iterator(processing_orders_.begin()),
      std::make_move_iterator(processing_orders_.end())
    );
    processing_orders_.clear();

    RCLCPP_INFO(rclcpp::get_logger("behavior"),
                "[Behavior] Moved %zu processing orders back to pending queue.",
                pending_orders_.size());
  }

  active_ = false;
  in_flight_ = false;
  current_state = IDLE;
  progress_ = {0, 0};
  note_ = "Cancelled";
}

// Retrieve the current status of the DeliveryRobot
BehaviorStatus DeliveryRobot::GetStatus() const {
  std::lock_guard<std::mutex> lock(mx_);
  BehaviorStatus status;
  status.active = active_;
  status.phase = phaseName(current_state);
  status.progress_current = progress_.first;
  status.progress_total = progress_.second;
  status.note = note_;
  return status;
}

// Add a new delivery order based on the provided ItemEntry
void DeliveryRobot::AddObject(ItemEntry item) {
  std::lock_guard<std::mutex> lock(mx_);
  if (active_) {
  // Create new object entry with default orientation (facing positive x)
  ObjectInfo new_order;
    unsigned int col = static_cast<unsigned int>(item.column);
    unsigned int row = static_cast<unsigned int>(item.row);

    if (col < 1) col = 1;
    if (col > 3) col = 3;
    if (row < 1) row = 1;
    if (row > 2) row = 2;

  double x_coord = kXBase + static_cast<double>(row - 1) * kXStep;
  double y_coord = kYBase + static_cast<double>(col - 1) * kYStep;

    new_order.pose.position.x = x_coord;
    new_order.pose.position.y = y_coord;
    new_order.pose.position.z = 0.0;
    new_order.pose.orientation.x = 0.0;
    new_order.pose.orientation.y = 0.0;
    new_order.pose.orientation.z = 0.0;
    new_order.pose.orientation.w = 1.0;
    new_order.type = std::string("unknown");
    new_order.item_id = static_cast<unsigned int>(item.id > 0 ? item.id : id_counter_++);
    pending_orders_.push_back(std::move(new_order));

  RCLCPP_INFO(rclcpp::get_logger("behavior"),
        "[Behavior] New object added: Item ID %d, Type %s at Grid (r=%u,c=%u) -> Loc (%.2f, %.2f).",
        new_order.item_id, new_order.type.c_str(), row, col,
        new_order.pose.position.x, new_order.pose.position.y);

  } else {
    RCLCPP_WARN(rclcpp::get_logger("behavior"),
                "[Behavior] Cannot add object; DeliveryRobot is not active.");
  }
}

// Retrieve list of objects based on the specified option
std::vector<ItemEntry> DeliveryRobot::GetObjects(ObjectOption option) const {
  std::lock_guard<std::mutex> lock(mx_);
  std::vector<ObjectInfo> object_infos_buffer;
  std::vector<ItemEntry> results;
  switch (option) {
    case IBaseRobot::ObjectOption::PENDING_OBJECTS:
      object_infos_buffer = {pending_orders_.begin(), pending_orders_.end()};
      break;
    case IBaseRobot::ObjectOption::COMPLETED_OBJECTS:
      object_infos_buffer = {completed_orders_.begin(), completed_orders_.end()};
      break;
    case IBaseRobot::ObjectOption::PROCESSING_OBJECTS:
      object_infos_buffer = {processing_orders_.begin(), processing_orders_.end()};
      break;
    default:
      object_infos_buffer = {completed_orders_.begin(), completed_orders_.end()};
      break;
  }

  results.reserve(object_infos_buffer.size());
  for (const auto& obj : object_infos_buffer) {
    ItemEntry entry;
    entry.id = obj.item_id;
  entry.row = static_cast<int32_t>(std::floor((obj.pose.position.x - kXBase) / kXStep + 1.0 + 1e-6));
  entry.column = static_cast<int32_t>(std::floor((obj.pose.position.y - kYBase) / kYStep + 1.0 + 1e-6));
    results.push_back(entry);
  }

  return results;
}

// Helper to convert state enum to string representation
std::string DeliveryRobot::phaseName(State s) const {
  switch (s) {
    case State::IDLE:
      return "IDLE";
    case State::SENDING_GOALS:
      return "SENDING_GOALS";
    case State::IN_PROGRESS:
      return "IN_PROGRESS";
    case State::COMPLETED:
      return "COMPLETED";
    default:
      return "UNKNOWN";
  }
}

// Sort orders based on their coordinates for optimal delivery route
std::vector<DeliveryRobot::ObjectInfo>
DeliveryRobot::sortOrders(const std::vector<ObjectInfo>& items) const {
  std::vector<ObjectInfo> sorted_items = items;

  std::sort(sorted_items.begin(), sorted_items.end(),
            [this](const ObjectInfo& a, const ObjectInfo& b) {
              if (std::abs(a.pose.position.x - b.pose.position.x) < COORDINATE_EPSILON)
                return a.pose.position.y < b.pose.position.y;
              return a.pose.position.x < b.pose.position.x;
            });

  return sorted_items;
}

// Extract poses from order information for navigation
std::vector<geometry_msgs::msg::Pose>
DeliveryRobot::extractPoses(const std::vector<ObjectInfo>& items) const {
  std::vector<geometry_msgs::msg::Pose> poses;
  
  for (const auto& order : items) {
    poses.push_back(order.pose);
  }

  return poses;
}

void DeliveryRobot::IDLEHandle() {
  {
    std::lock_guard<std::mutex> lock(mx_);
    RCLCPP_INFO(rclcpp::get_logger("behavior"),
                "[Behavior] State=IDLE pending=%zu processing=%zu completed=%zu in_flight=%s",
                pending_orders_.size(), processing_orders_.size(), completed_orders_.size(),
                in_flight_ ? "true" : "false");
  }
  if (!pending_orders_.empty()) {
      next_state = SENDING_GOALS;
    }
}

void DeliveryRobot::SENDING_GOALSHandle() {
  {
    std::lock_guard<std::mutex> lock(mx_);
    RCLCPP_INFO(rclcpp::get_logger("behavior"),
                "[Behavior] State=SENDING_GOALS pending=%zu", pending_orders_.size());
  }
  bool server_ok = (act_.query_server() == RobotPositionManagerClient::ServerStatus::Available);
  
  if (!server_ok) {
    std::lock_guard<std::mutex> lock(mx_);
    note_ = "Navigator Busy, retry later";
    next_state = SENDING_GOALS;
    return;
  }
  
  std::vector<ObjectInfo> local_pending_orders;
  std::vector<geometry_msgs::msg::Pose> goal_coordinates;
  
  {
    std::lock_guard<std::mutex> lock(mx_);
    if (pending_orders_.empty()) {
      next_state = IDLE;
      return;
    }
    local_pending_orders = std::move(pending_orders_);
  }
  
  local_pending_orders = sortOrders(local_pending_orders);
  goal_coordinates = extractPoses(local_pending_orders);
  
  try {
    future_ = act_.navigate(
      goal_coordinates,
      [this](int current, int total) {
        std::lock_guard<std::mutex> g(mx_);
        progress_ = {current, total};
        one_item_delivered_ = true;
        RCLCPP_INFO(rclcpp::get_logger("behavior"),
                    "[Behavior] new goal reached: (%d/%d)", current, total);
      }
    );
    
    {
      std::lock_guard<std::mutex> lock(mx_);
      in_flight_ = true;
      progress_ = {0, static_cast<int>(goal_coordinates.size())};
      note_ = "Navigation started";
      processing_orders_ = std::move(local_pending_orders);
      next_state = IN_PROGRESS;
    }
  }
  catch (const std::exception& e) {
    std::lock_guard<std::mutex> lock(mx_);
    RCLCPP_ERROR(rclcpp::get_logger("behavior"), "[Behavior] Navigation failed: %s", e.what());
    in_flight_ = false;
    note_ = "Navigation failed";
    pending_orders_ = std::move(local_pending_orders);
    next_state = IDLE;
  }
}

void DeliveryRobot::IN_PROGRESSHandle() {
  {
    std::lock_guard<std::mutex> lock(mx_);
    RCLCPP_INFO(rclcpp::get_logger("behavior"),
                "[Behavior] State=IN_PROGRESS progress=(%d/%d) queue=%zu",
                progress_.first, progress_.second, processing_orders_.size());
  }
  if (future_.valid()) {
    auto status = future_.wait_for(std::chrono::seconds(0));
    
    if (status == std::future_status::ready) {
      try {
        auto result = future_.get();
        std::lock_guard<std::mutex> lock(mx_);
        
        if (result.success) {
          RCLCPP_INFO(rclcpp::get_logger("behavior"),
                    "[Behavior] Navigation completed successfully");
          next_state = COMPLETED;
          return;
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("behavior"),
                     "[Behavior] Navigation failed with status: %d", 
                     result.completed);
          pending_orders_.insert(
            pending_orders_.end(),
            std::make_move_iterator(processing_orders_.begin()),
            std::make_move_iterator(processing_orders_.end())
          );
          processing_orders_.clear();
          in_flight_ = false;
          next_state = IDLE;
          return;
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("behavior"),
                   "[Behavior] Exception while getting navigation result: %s", 
                   e.what());
        next_state = IDLE;
        return;
      }
    }
  }

  if (one_item_delivered_) {
    std::lock_guard<std::mutex> lock(mx_);
    one_item_delivered_ = false;
    
    if (!processing_orders_.empty()) {
      ObjectInfo delivered_order = processing_orders_.front();
      completed_orders_.push_back(delivered_order);
      processing_orders_.erase(processing_orders_.begin());

  RCLCPP_INFO(rclcpp::get_logger("behavior"),
      "[Behavior] Object delivered: Item ID %d, Type %s at Location (%.2f, %.2f).",
      delivered_order.item_id, delivered_order.type.c_str(),
      delivered_order.pose.position.x, delivered_order.pose.position.y);
                  
      note_ = "Delivered order " + std::to_string(delivered_order.item_id);
    }
  }
}

void DeliveryRobot::COMPLETEDHandle() {
  {
    std::lock_guard<std::mutex> lock(mx_);
    RCLCPP_INFO(rclcpp::get_logger("behavior"),
                "[Behavior] State=COMPLETED delivered_total=%zu", completed_orders_.size());
  }
  {
    std::lock_guard<std::mutex> lock(mx_);
    in_flight_ = false;
    completed_orders_.insert(
      completed_orders_.end(),
      std::make_move_iterator(processing_orders_.begin()),
      std::make_move_iterator(processing_orders_.end())
    );
    processing_orders_.clear();
    note_ = "All orders completed";
    next_state = IDLE;
  }

  try {
    std::vector<geometry_msgs::msg::Pose> wps{ return_pose_ };
    RCLCPP_INFO(rclcpp::get_logger("behavior"), "[Behavior] Navigating to return pose (%.2f, %.2f)",
                return_pose_.position.x, return_pose_.position.y);
    future_ = act_.navigate(wps, nullptr);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("behavior"), "[Behavior] Failed to navigate to return pose: %s", e.what());
  }
}


void DeliveryRobot::tick() {
  if (!active_) {
    return;
  } 

  if (last_state_ != current_state) {
    RCLCPP_INFO(rclcpp::get_logger("behavior"),
                "[Behavior] State transition: %s -> %s",
                phaseName(last_state_).c_str(), phaseName(current_state).c_str());
    last_state_ = current_state;
  }

  current_state = next_state;

  switch (current_state)
  {
  case IDLE:
    IDLEHandle();
    break;
  case SENDING_GOALS:
    SENDING_GOALSHandle();
    break;
  case IN_PROGRESS:
    IN_PROGRESSHandle();
    break;
  case COMPLETED:
    COMPLETEDHandle();
    break;
  default:
    break;
  }
}
