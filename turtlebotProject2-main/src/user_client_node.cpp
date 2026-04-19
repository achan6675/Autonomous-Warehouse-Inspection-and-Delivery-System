#include "user_client_node.hpp"

/**
 * @brief Constructor that initializes the user client node and seeds the random generator.
 */
UserClientNode::UserClientNode() : Node("user_client_node") {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));  // seed random generator

}

/**
 * @brief Sends a request to retrieve a text file from the robot and processes the response.
 * @param file_type Type of file to request ("delivery" or "inspection").
 */
void UserClientNode::send_text_request(const std::string &file_type) {
    std::string service_name;
    if (file_type == "delivery") {
        service_name = "get_delivery_file";
    } else if (file_type == "inspection") {
        service_name = "get_damage_file";
    } else {
        RCLCPP_ERROR(get_logger(), "Invalid file type '%s'. Expected 'delivery' or 'inspection'.", file_type.c_str());
        return;
    }

    text_client_ = this->create_client<GetTextFile>(service_name);

    auto request = std::make_shared<GetTextFile::Request>();
    request->file_type = file_type;

    while (!text_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    auto future = text_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(shared_from_this(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();

        if (file_type == "delivery") {
            deliveryEntries.clear();
            deliveryEntries.reserve(response->entries.size());
            for (const auto &ros_entry : response->entries) {
                ItemEntry entry;
                entry.id = ros_entry.id;
                entry.row = ros_entry.row;
                entry.column = ros_entry.column;
                deliveryEntries.push_back(entry);
            }
            textFileGenerator.GetDeliveryFile(deliveryEntries);
        } else if (file_type == "inspection") {
            damageEntries.clear();
            damageEntries.reserve(response->entries.size());
            for (const auto &ros_entry : response->entries) {
                ItemEntry entry;
                entry.id = ros_entry.id;
                entry.row = ros_entry.row;
                entry.column = ros_entry.column;
                damageEntries.push_back(entry);
            }
            textFileGenerator.GetDamageFile(damageEntries);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid file type requested");
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /get_text_file");
    }
}

/**
 * @brief Sends a move request to the robot for delivery or inspection and handles the response.
 * @param action_type Type of move request ("delivery" or "inspection").
 */
void UserClientNode::send_move_request(const std::string &action_type) {
    move_client_ = this->create_client<MoveTo>("move_to");

    while (!move_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for robot to move...");
        if (!rclcpp::ok()) return;
    }

    auto request = std::make_shared<MoveTo::Request>();
    if (action_type == "delivery") {
        request->action_type = "delivery";    
    }
    if (action_type == "inspection") {
        request->action_type = "inspection"; 
    }

    request->target_row = std::rand() % 4;
    request->target_column = std::rand() % 4;

    auto future = move_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "User node: %s", response->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Move request failed!");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call Move service.");
    }
}

/**
 * @brief Reports the current Battery Percentage to the user
 */
void UserClientNode::get_battery() {

    battery_client_ = this->create_client<GetBattery>("get_battery");


    while (!battery_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for battery service");
        if (!rclcpp::ok()) return;
    }

    auto request = std::make_shared<GetBattery::Request>();

    auto future = battery_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Battery Percentage: %.2f", response->percent);
        
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call Battery service.");
    }
    

}

/**
 * @brief Entry point for the user client node executable.
 */int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UserClientNode>();

    std::string input_type;
    std::string robot_type;

    while (rclcpp::ok()) {
        std::cout << "\nEnter request type (textfile/request/charge) or 'exit' to quit: ";
        std::cin >> input_type;
        if (input_type == "exit") {break;}

        std::cout << "Enter robot type (delivery/inspection): ";
        std::cin >> robot_type;

        if (input_type == "textfile") {
            node->send_text_request(robot_type);
        } else if (input_type == "request") {
            node->send_move_request(robot_type);
        } else if (input_type == "charge") {
            node->get_battery();
        } else {
            std::cout << "Invalid request type.\n";
        }

        // Process any incoming messages (e.g., battery updates)
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}

