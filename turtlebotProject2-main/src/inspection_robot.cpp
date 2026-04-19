#include "turtlebot3_gazebo/inspection_robot.hpp"

#include <cmath>
#include <sstream>

namespace turtlebot3_gazebo {

// Constructor initializes the damage point manager with idle state and
// cleared status fields.
InspectionRobot::InspectionRobot() : IBehavior("InspectionRobot") {
  std::lock_guard<std::mutex> lk(mu_);
  scan_state_ = ScanState::Idle;
  status_.active = false;
  status_.phase = "idle";
  status_.progress_current = 0;
  status_.progress_total = 0;
  status_.note.clear();
}

InspectionRobot::~InspectionRobot() = default;

// Resets all internal state including damage lists, item entries, and state
// machine to Idle. Called when starting a new scanning session.
void InspectionRobot::start() {
  std::lock_guard<std::mutex> lk(mu_);

  damaged_list_.clear();
  item_entries_.clear();
  next_id_ = 1;

  scan_state_ = ScanState::Idle;

  status_.active = false;
  status_.phase = "idle";
  status_.progress_current = 0;
  status_.progress_total = 0;
  status_.note.clear();
}

// Cancels the current operation, clears damage list, and returns to Idle state.
// Sets status note to "cancelled" for debugging purposes.
void InspectionRobot::cancel() {
  std::lock_guard<std::mutex> lk(mu_);

  damaged_list_.clear();
  scan_state_ = ScanState::Idle;

  status_.active = false;
  status_.phase = "idle";
  status_.note = "cancelled";
}

// Returns the current behavior status including active state, phase, progress,
// and debug notes. Thread-safe access with mutex lock.
BehaviorStatus InspectionRobot::GetStatus() const {
  std::lock_guard<std::mutex> lk(mu_);
  return status_;
}

// Callback for robot state updates. Currently not used in this behavior
// implementation as the manager doesn't depend on robot state.
void InspectionRobot::onState(const RobotState&) {
  // Not dependent on robot state currently
}

// Initiates the scanning phase by transitioning to Scanning state. Sets
// status to active and updates phase and note fields.
void InspectionRobot::begin_scan() {
  std::lock_guard<std::mutex> lk(mu_);
  scan_state_ = ScanState::Scanning;
  status_.active = true;
  status_.phase = "scanning";
  status_.note = "scanning map";
}

// Marks the scanning phase as complete and transitions to SendGoal state,
// indicating readiness to send navigation goals.
void InspectionRobot::mark_scan_done() {
  std::lock_guard<std::mutex> lk(mu_);
  scan_state_ = ScanState::SendGoal;
}

// Transitions to Processing state, indicating that navigation is in progress.
// Updates status phase and note accordingly.
void InspectionRobot::send_goal() {
  std::lock_guard<std::mutex> lk(mu_);
  scan_state_ = ScanState::Processing;
  status_.phase = "processing";
  status_.note = "navigation in progress";
}

// Binds ROS2 service servers to the provided node for controlling the scanning
// workflow: begin_scan, mark_scan_done, and send_goal services.
void InspectionRobot::bind_services(const rclcpp::Node::SharedPtr& node) {
  using Trigger = std_srvs::srv::Trigger;

  srv_begin_scan_ = node->create_service<Trigger>(
      "damage_point/begin_scan",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> res) {
        this->begin_scan();
        res->success = true;
        res->message = "begin_scan triggered";
      });

  srv_mark_done_ = node->create_service<Trigger>(
      "damage_point/mark_scan_done",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> res) {
        this->mark_scan_done();
        res->success = true;
        res->message = "mark_scan_done triggered";
      });

  srv_send_goal_ = node->create_service<Trigger>(
      "damage_point/send_goal", [this](const std::shared_ptr<Trigger::Request>,
                                       std::shared_ptr<Trigger::Response> res) {
        this->send_goal();
        res->success = true;
        res->message = "send_goal triggered";
      });
}

// Adds a damage point from ItemEntry (grid coordinates). Creates corresponding
// ObjectInfo with placeholder pose and stores both representations.
void InspectionRobot::AddObject(ItemEntry item) {
  std::lock_guard<std::mutex> lk(mu_);

  // Store directly in item_entries_
  item_entries_.push_back(item);

  // Also create ObjectInfo for internal tracking
  IBehavior::ObjectInfo obj;
  obj.pose.position.x = 0.0;
  obj.pose.position.y = 0.0;
  obj.type = "damage point";
  obj.item_id = next_id_;

  damaged_list_.push_back(obj);

  if (scan_state_ == ScanState::Idle) {
    scan_state_ = ScanState::Scanning;
  }

  status_.active = true;
  status_.phase = "scanning";
  status_.progress_current = 0;
  status_.progress_total = static_cast<int>(damaged_list_.size());
  status_.note = "object added";

  ++next_id_;
}

// Adds a damage point from QR code position. Converts X coordinate to grid row
// using binning algorithm and stores both ObjectInfo (with pose) and ItemEntry.
void InspectionRobot::AddObjectFromQR(
    geometry_msgs::msg::PointStamped qr_position) {
  std::lock_guard<std::mutex> lk(mu_);

  IBehavior::ObjectInfo obj;
  obj.pose.position.x = qr_position.point.x;
  obj.pose.position.y = qr_position.point.y;
  obj.type = "damage point";
  obj.item_id = next_id_;

  damaged_list_.push_back(obj);

  if (scan_state_ == ScanState::Idle) {
    scan_state_ = ScanState::Scanning;
  }

  const double halfL = MAP_LENGTH_M * 0.5;
  const double band = halfL / static_cast<double>(NUM_ROWS);
  const double x = qr_position.point.x;
  const double dist = std::abs(x);

  int row = static_cast<int>(std::floor(dist / band)) + 1;
  if (row < 1) row = 1;
  if (row > NUM_ROWS) row = NUM_ROWS;

  turtlebot3_gazebo::msg::ItemEntry entry;
  entry.column = static_cast<std::uint8_t>(FIXED_COLUMN);
  entry.row = static_cast<std::uint8_t>(row);
  item_entries_.push_back(entry);

  status_.active = true;
  status_.phase = "scanning";
  status_.progress_current = 0;
  status_.progress_total = static_cast<int>(damaged_list_.size());
  status_.note = "object added";

  ++next_id_;
}

// State machine tick function. Updates status fields based on current scan
// state: Idle, Scanning, SendGoal, Processing, or Finished.
void InspectionRobot::tick() {
  std::lock_guard<std::mutex> lk(mu_);

  switch (scan_state_) {
    case ScanState::Idle: {
      if (damaged_list_.empty()) {
        status_.active = false;
        status_.phase = "idle";
        status_.progress_current = 0;
        status_.progress_total = 0;
        status_.note.clear();
      } else {
        status_.active = true;
        status_.phase = "pending";
        status_.progress_current = 0;
        status_.progress_total = static_cast<int>(damaged_list_.size());
        status_.note = "waiting to start scanning";
      }
      return;
    }

    case ScanState::Scanning: {
      status_.active = true;
      status_.phase = "scanning";
      status_.progress_current = 0;
      status_.progress_total = static_cast<int>(damaged_list_.size());
      status_.note = "scanning map";
      return;
    }

    case ScanState::SendGoal: {
      status_.active = true;
      status_.phase = "ready_to_send";
      status_.progress_current = 0;
      status_.progress_total = static_cast<int>(damaged_list_.size());
      status_.note = "scan complete, ready to send goal";
      return;
    }

    case ScanState::Processing: {
      status_.active = true;
      status_.phase = "processing";
      status_.progress_current = 0;
      status_.progress_total = static_cast<int>(damaged_list_.size());
      status_.note = "navigation in progress";
      return;
    }

    case ScanState::Finished: {
      status_.active = false;
      status_.phase = "finished";
      status_.progress_current = static_cast<int>(damaged_list_.size());
      status_.progress_total = static_cast<int>(damaged_list_.size());
      status_.note = "all tasks completed";
      return;
    }
  }
}

// Returns damage points as ItemEntry list (grid coordinates) filtered by
// option: pending, completed, or processing based on current state.
std::vector<ItemEntry> InspectionRobot::GetObjects(
    IBehavior::ObjectOption option) const {
  std::lock_guard<std::mutex> lk(mu_);

  switch (option) {
    case IBehavior::PENDING_OBJECTS:
      if (scan_state_ != ScanState::Finished) {
        return item_entries_;
      }
      break;
    case IBehavior::COMPLETED_OBJECTS:
      if (scan_state_ == ScanState::Finished) {
        return item_entries_;
      }
      break;
    case IBehavior::PROCESSING_OBJECTS:
      if (scan_state_ == ScanState::Processing) {
        return item_entries_;
      }
      break;
    default:
      break;
  }
  return {};
}

// Returns damage points as ObjectInfo list (with pose and metadata) filtered
// by option: pending, completed, or processing based on current state.
std::vector<IBehavior::ObjectInfo> InspectionRobot::GetObjectsAsInfo(
    IBehavior::ObjectOption option) const {
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<IBehavior::ObjectInfo> out;

  switch (option) {
    case IBehavior::PENDING_OBJECTS:
      if (scan_state_ != ScanState::Finished) {
        out = damaged_list_;
      }
      break;
    case IBehavior::COMPLETED_OBJECTS:
      if (scan_state_ == ScanState::Finished) {
        out = damaged_list_;
      }
      break;
    case IBehavior::PROCESSING_OBJECTS:
      if (scan_state_ == ScanState::Processing) {
        out = damaged_list_;
      }
      break;
    default:
      break;
  }
  return out;
}

// Sets the return pose for the robot. Required by IBehavior interface but
// not used in this damage point behavior implementation.
void InspectionRobot::set_return_pose(const geometry_msgs::msg::Pose&) {
  // Not used in this behavior
}

// Returns const reference to the internal ItemEntry list for efficient
// read-only access without copying.
const std::vector<turtlebot3_gazebo::msg::ItemEntry>&
InspectionRobot::get_item_entries() const {
  return item_entries_;
}

// Returns the current scan state machine state. Thread-safe with mutex lock.
InspectionRobot::ScanState InspectionRobot::scan_state() const {
  std::lock_guard<std::mutex> lk(mu_);
  return scan_state_;
}

// Generates a debug summary string with current state name, damaged point
// count, and item entry count for quick inspection and debugging.
std::string InspectionRobot::dump_summary() const {
  std::lock_guard<std::mutex> lk(mu_);
  std::ostringstream oss;

  auto state_str = [this]() -> const char* {
    switch (scan_state_) {
      case ScanState::Idle:
        return "Idle";
      case ScanState::Scanning:
        return "Scanning";
      case ScanState::SendGoal:
        return "SendGoal";
      case ScanState::Processing:
        return "Processing";
      case ScanState::Finished:
        return "Finished";
    }
    return "Unknown";
  }();

  oss << "state=" << state_str << ", damaged_count=" << damaged_list_.size()
      << ", item_entries=" << item_entries_.size();
  return oss.str();
}

}  // namespace turtlebot3_gazebo
