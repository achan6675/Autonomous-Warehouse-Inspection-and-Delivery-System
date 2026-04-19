#include "robot_location_manager_node.hpp"

/**
 * @brief Constructor that initializes the move and text file service servers.
 */
RobotLocationManagerNode::RobotLocationManagerNode()
    : Node("robot_location_manager_node")
{
    move_server = this->create_service<MoveTo>(
        "move_to",
        std::bind(&RobotLocationManagerNode::move_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    text_server = this->create_service<GetTextFile>(
        "get_delivery_file",
        std::bind(&RobotLocationManagerNode::text_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
}

/**
 * @brief Handles move requests for charging, delivery, or inspection and responds with status.
 * @param request Shared pointer to the MoveTo service request.
 * @param response Shared pointer to the MoveTo service response.
 */
void RobotLocationManagerNode::move_callback(
    const std::shared_ptr<MoveTo::Request> request,
    std::shared_ptr<MoveTo::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Recharge request received.");

    if (request->action_type == "charge") {
        response->success = true;
        response->message = "Moving to charging dock at (0,0)";
    }
    else if (request->action_type == "delivery") {
        response->success = true;
        response->message = "Delivering package at location (" +
                            std::to_string(request->target_row) + "," +
                            std::to_string(request->target_column) + ")";

        project_2::msg::ItemEntry entry;
        entry.id = next_id_++;
        entry.row = request->target_row;
        entry.column = request->target_column;
        DeliveryEntries.push_back(entry);
    }
    else if (request->action_type == "inspection") {
        response->success = true;
        response->message = "Inspecting damages at location (" +
                            std::to_string(request->target_row) + "," +
                            std::to_string(request->target_column) + ")";
    }

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

/**
 * @brief Responds to requests for delivery information by sending all stored entries.
 * @param request Shared pointer to the GetTextFile service request.
 * @param response Shared pointer to the GetTextFile service response.
 */
void RobotLocationManagerNode::text_callback(
    const GetTextFile::Request::SharedPtr request,
    const GetTextFile::Response::SharedPtr response)
{
    RCLCPP_INFO(get_logger(), "A request has been made");

    for (const auto &entry : DeliveryEntries) {
        response->entries.push_back(entry);
    }
}

/**
 * @brief Entry point for the robot location manager node executable.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotLocationManagerNode>());
    rclcpp::shutdown();
    return 0;
}
