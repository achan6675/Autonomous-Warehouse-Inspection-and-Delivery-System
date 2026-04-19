#include "battery_life_node.hpp"

/**
 * @brief Constructor that initializes the battery life node and its service client and subscriber.
 */
BatteryLifeNode::BatteryLifeNode() : Node("battery_life_node") {
    client_ = this->create_client<MoveTo>("move_to");

    battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "battery_state",
        rclcpp::SensorDataQoS(),
        std::bind(&BatteryLifeNode::battery_status_callback, this, std::placeholders::_1)
    );

     battery_server = this->create_service<GetBattery>(
        "get_battery",
        std::bind(&BatteryLifeNode::get_battery, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/**
 * @brief Callback that monitors battery percentage and triggers a recharge request if low.
 * @param msg Shared pointer to the incoming battery state message.
 */
void BatteryLifeNode::battery_status_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    *current_battery_percentage = msg->percentage;

    if (*current_battery_percentage < 0.1) {
        RCLCPP_WARN(this->get_logger(),
            "Battery low (%.1f%%)! Sending recharge request...", current_battery_percentage);

        send_recharge_request();
    }
}

void BatteryLifeNode::get_battery(
    const GetBattery::Request::SharedPtr request,
    const GetBattery::Response::SharedPtr response)
{
    RCLCPP_INFO(get_logger(), "A request has been made");

        response->percent = *current_battery_percentage;
    
}

/**
 * @brief Sends a request to the robot to move to the charging dock.
 */
void BatteryLifeNode::send_recharge_request() {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for recharge service...");
        if (!rclcpp::ok()) return;
    }

    auto request = std::make_shared<MoveTo::Request>();
    request->action_type = "charge";
    request->target_row = 0;
    request->target_column = 0;

    auto future = client_->async_send_request(request);

    rclcpp::executors::SingleThreadedExecutor exec;
    if (exec.spin_until_future_complete(future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Battery node: %s", response->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Recharge request failed!");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call recharge service.");
    }
}

/**
 * @brief Entry point for the battery life node executable.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryLifeNode>());
    rclcpp::shutdown();
    return 0;
}
