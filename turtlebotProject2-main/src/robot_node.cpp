#include "turtlebot3_gazebo/robot_node.hpp"
#include "turtlebot3_gazebo/IBaseRobot.hpp"
#include <chrono>
#include "turtlebot3_gazebo/srv/get_battery.hpp"

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: robot_node.cpp
// Author(s): Yunkan Luo
//
// Implements the RobotNode which wires together the delivery behaviour,
// RobotPositionManagerClient and exposes services like /move_to and /get_delivery_file.


// ---------------------- RobotNode Implementation ----------------------

RobotNode::RobotNode() : Node("robot_node")
{
  delivery_ = std::make_shared<DeliveryRobot>();
  delivery_->start();
  // Configure the return/home pose for the delivery manager
  geometry_msgs::msg::Pose return_pose;
  return_pose.position.x = -0.386;
  return_pose.position.y = -0.374;
  return_pose.position.z = 0.0;
  return_pose.orientation.w = 1.0;
  delivery_->set_return_pose(return_pose);

  // Set timer to call tick() every 100ms
  using namespace std::chrono_literals;
  tick_timer_ = this->create_wall_timer(100ms, [this]() {
    if (delivery_) delivery_->tick();
  });

  // create /move_to service (request: MoveTo.srv)
  move_to_srv_ = this->create_service<MoveTo>(
    "move_to",
    [this](
      const std::shared_ptr<MoveTo::Request> req,
      std::shared_ptr<MoveTo::Response> res)
    {
      if (!delivery_) {
        res->success = false;
        res->message = "Delivery behavior not ready";
        RCLCPP_WARN(this->get_logger(), "[move_to] rejected: delivery_ null");
        return;
      }

      try {
        // Validate ranges: column (x) in [1,3], row (y) in [1,2]
        if (req->target_column < 1 || req->target_column > 3) {
          res->success = false;
          res->message = "target_column out of range (1..3)";
          RCLCPP_WARN(this->get_logger(), "[move_to] rejected: column %u out of range", req->target_column);
          return;
        }
        if (req->target_row < 1 || req->target_row > 2) {
          res->success = false;
          res->message = "target_row out of range (1..2)";
          RCLCPP_WARN(this->get_logger(), "[move_to] rejected: row %u out of range", req->target_row);
          return;
        }

        // Construct ItemEntry using the current message layout: id,row,column
        // requested grid indices and leave type handling to the manager.
        ItemEntryMsg item;
        item.id = 0; // 0 = unspecified; manager may assign a unique id
        item.row = static_cast<int32_t>(req->target_row);
        item.column = static_cast<int32_t>(req->target_column);

        delivery_->AddObject(item);
        delivery_->tick();

        res->success = true;
        res->message = "MoveTo accepted";
        // Also log the mapped world coordinates (same mapping used in the manager)
        // new mapping (row -> x, column -> y):
        // x = 0.5 + (row-1)*(3.25/2) ; y = -0.2 + (column-1)*0.2
        constexpr double x_step = 3.25 / 2.0;
        constexpr double y_base = -0.2;
        constexpr double y_step = 0.2;
        double world_x = 0.5 + (static_cast<double>(item.row) - 1.0) * x_step;
        double world_y = y_base + (static_cast<double>(item.column) - 1.0) * y_step;
        RCLCPP_INFO(this->get_logger(),
          "[move_to] accepted: row=%d col=%d -> world (x=%.3f,y=%.3f)",
          item.row, item.column, world_x, world_y);
      } catch (const std::exception &e) {
        res->success = false;
        res->message = std::string("Exception: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "[move_to] exception: %s", e.what());
      }
    }
  );

  RCLCPP_INFO(this->get_logger(), "Service [/move_to] is ready");

  // Create battery client and periodic checker
  battery_client_ = this->create_client<turtlebot3_gazebo::srv::GetBattery>("get_battery");

  using namespace std::chrono_literals;
  // check battery every 1 second
  battery_timer_ = this->create_wall_timer(1s, [this]() {
    if (!rclcpp::ok()) return;

    // Wait briefly for service availability
    if (!battery_client_->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_DEBUG(this->get_logger(), "Battery service not available yet");
      return;
    }

    auto req = std::make_shared<turtlebot3_gazebo::srv::GetBattery::Request>();
    auto fut = battery_client_->async_send_request(req);
    // wait for up to 500ms for response
    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, std::chrono::milliseconds(500));
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Failed to call battery service (timeout or error)");
      return;
    }

    auto res = fut.get();
    float percent = res->percent;
    RCLCPP_DEBUG(this->get_logger(), "Battery check: %.3f", percent);

    // If battery below threshold, cancel delivery and send robot to home
    constexpr float kLowBatteryThreshold = 0.2f;
    if (percent < kLowBatteryThreshold) {
      RCLCPP_WARN(this->get_logger(), "Low battery: %.3f (< %.3f). Cancelling delivery and returning home.", percent, kLowBatteryThreshold);

      // Cancel any delivery behavior in progress
      if (delivery_) {
        try {
          delivery_->cancel();
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Exception while cancelling delivery: %s", e.what());
        }
      }

      // Try to stop current navigator goal and send a return-home waypoint
      auto &bn = RobotPositionManagerClient::getInstance();
      try {
        // cancel any running navigation goal first
        bn.cancel_all();

        // Define home pose (use the same return pose configured on the manager)
        geometry_msgs::msg::Pose home_pose;
        home_pose.position.x = -0.386;  // configured home X
        home_pose.position.y = -0.374;  // configured home Y
        home_pose.position.z = 0.0;
        // identity orientation
        home_pose.orientation.w = 1.0;

        std::vector<geometry_msgs::msg::Pose> wps{home_pose};

        // send navigation to home (non-blocking). we don't need feedback here.
        auto fut_res = bn.navigate(wps, nullptr);
        // Optionally wait a short time for the navigator to accept the goal
        // but don't block the timer for long.
        // Note: if navigate throws because not initialized, catch and log.
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send return-home to RobotPositionManagerClient: %s", e.what());
      }
    }
  });

  text_file_srv_ = this->create_service<GetTextFile>(
  "get_delivery_file",
  [this](
    const std::shared_ptr<GetTextFile::Request> /*req*/,
    std::shared_ptr<GetTextFile::Response> res)
    {
      if (!delivery_) {
        RCLCPP_ERROR(this->get_logger(),
                    "[get_text_file] delivery_ is null, cannot fetch entries.");
        res->entries.clear();
        return;
      }
      std::vector<ItemEntryMsg> items = delivery_->GetObjects(IBaseRobot::COMPLETED_OBJECTS);

      res->entries = std::move(items);

      RCLCPP_INFO(this->get_logger(),
                  "[get_text_file] served request for completed orders, count=%zu",
                  res->entries.size());
    }
  );

  RCLCPP_INFO(this->get_logger(), "Service [/get_delivery_file] is ready");
}

// Bind the RobotPositionManagerClient to the delivery robot's actions
void RobotNode::wire_navigator(RobotPositionManagerClient &bn)
{
  IBaseRobot::Act act;
  act.navigate = [&bn](
                  const std::vector<geometry_msgs::msg::Pose> & wps,
                  std::function<void(int,int)> user_on_feedback
  ) -> std::shared_future<RobotPositionManagerClient::ResultSummary> {
    if (user_on_feedback) {
      RCLCPP_INFO(rclcpp::get_logger("robot_node"), "Navigating with user feedback callback.");
      return bn.navigate(wps, std::move(user_on_feedback));
    } else {
      return bn.navigate(wps, [](int,int){});
    }
  };
  act.cancel_all = [&bn]() { bn.cancel_all(); };
  act.query_server = [&bn]() -> RobotPositionManagerClient::ServerStatus { return bn.get_status(); };

  delivery_->bind_actions(act);
  RCLCPP_INFO(this->get_logger(), "Delivery actions bound to navigator.");
}

// ---------------------- main ----------------------

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotNode>();

  auto &bn = RobotPositionManagerClient::getInstance();
  bn.initialize(node, "batch_navigate");

  node->wire_navigator(bn);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}