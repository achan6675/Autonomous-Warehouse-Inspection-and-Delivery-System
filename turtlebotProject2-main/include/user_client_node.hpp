// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: user_client_node.hpp
// Author(s): Amelia Chan
//
// Node that sends service requests to the robot for moving and retrieving text files
// containing delivery or damage information.

#ifndef USER_CLIENT_NODE_HPP_
#define USER_CLIENT_NODE_HPP_

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include <memory>
#include <vector> 

#include "project_2/srv/get_text_file.hpp"
#include "project_2/srv/move_to.hpp"
#include "project_2/msg/item_entry.hpp"
#include "project_2/srv/get_battery.hpp"


#include "text_file_generator.hpp"

using project_2::srv::GetTextFile;
using project_2::srv::MoveTo;
using project_2::srv::GetBattery;


/**
 * @brief Node for sending requests to the robot for movements and file retrievals.
 */
class UserClientNode : public rclcpp::Node {
public: 
    /**
     * @brief Constructor for the UserClientNode.
     * Initializes clients for move and text file services.
     */
    UserClientNode();

    /**
     * @brief Destructor for the UserClientNode.
     * Default destructor.
     */
    ~UserClientNode(){};

    /**
     * @brief Sends a request to get a text file from the robot.
     * @param file_type Type of file to request (e.g., "delivery" or "damage").
     */
    void send_text_request(const std::string &file_type);

    /**
     * @brief Sends a request for the robot to move to a location.
     * @param file_type Type of move request (may be used to select move behavior).
     */
    void send_move_request(const std::string &file_type);

      /**
     * @brief Sends a request for the robot's current battery percentage
     */
    void get_battery();

private: 
    /// Object for generating and writing text files.
    TextFileGenerator textFileGenerator;

    /// Stores received delivery information from the robot.
    std::vector<ItemEntry> deliveryEntries;

    /// Stores received damage information from the robot.
    std::vector<ItemEntry> damageEntries;

    /// Client for requesting text files from the robot.
    rclcpp::Client<GetTextFile>::SharedPtr text_client_;

    /// Client for sending move commands to the robot.
    rclcpp::Client<MoveTo>::SharedPtr move_client_;

    rclcpp::Client<GetBattery>::SharedPtr battery_client_;


    
    

};

#endif
