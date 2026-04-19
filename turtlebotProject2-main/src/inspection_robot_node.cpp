#include "inspection_robot_node.hpp"

/**
 * @brief Constructor that initializes publishers, subscribers, timers, and services.
 */
InspectionRobotNode::InspectionRobotNode() : rclcpp::Node("inspection_robot_node") {
    /************************************************************
     ** Initialise ROS publishers and subscribers
    ************************************************************/
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Initialise publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", qos);

    /************************************************************
     ** Initialise ROS timers
    ************************************************************/
    update_timer_ = this->create_wall_timer(
        5000ms,
        std::bind(&InspectionRobotNode::update_callback, this)
    );

    // Initialise service
    server = this->create_service<GetTextFile>(
        "get_damage_file",
        std::bind(&InspectionRobotNode::service_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Initialise subscriber
    subscription_ = this->create_subscription<project_2::msg::ItemEntry>(
        "damages_found", 10,
        std::bind(&InspectionRobotNode::damage_callback, this, std::placeholders::_1)
    );
}

/**
 * @brief Callback that stores detected damage entries received from the camera.
 * @param entry Shared pointer to a detected damage message.
 */
void InspectionRobotNode::damage_callback(const project_2::msg::ItemEntry::SharedPtr entry) {
    RCLCPP_INFO(get_logger(),
        "inspection robot control node received detected damage: id=%d (row=%d, column=%d)",
        entry->id, entry->row, entry->column);

    DamageEntries.push_back(*entry);
}

/**
 * @brief Service callback that returns all stored damage entries.
 * @param request Shared pointer to the service request.
 * @param response Shared pointer to the service response.
 */
void InspectionRobotNode::service_callback(
    const GetTextFile::Request::SharedPtr request,
    const GetTextFile::Response::SharedPtr response) 
{
    RCLCPP_INFO(get_logger(), "A request has been made");

    for (const auto &entry : DamageEntries) {
        response->entries.push_back(entry);
    }
}

/**
 * @brief Publishes a velocity command to the /cmd_vel topic.
 * @param linear Linear velocity to publish.
 * @param angular Angular velocity to publish.
 */
void InspectionRobotNode::update_cmd_vel(double linear, double angular) {
    geometry_msgs::msg::TwistStamped cmd_vel_stamped;
    cmd_vel_stamped.header.stamp = this->get_clock()->now();
    cmd_vel_stamped.twist.linear.x = linear;
    cmd_vel_stamped.twist.angular.z = angular;

    cmd_vel_pub_->publish(cmd_vel_stamped);
}

/**
 * @brief Timer callback that computes the next movement command and publishes it.
 */
void InspectionRobotNode::update_callback() {
    // Compute the next movement command (linear + angular velocities)
    CommandVelocity cmd = inspectionnavigator.move();

    // Publish the resulting command
    update_cmd_vel(cmd.linear, cmd.angular);
}

/**
 * @brief Default destructor.
 */
InspectionRobotNode::~InspectionRobotNode() = default;

/**
 * @brief Entry point for the inspection robot node executable.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InspectionRobotNode>());
    rclcpp::shutdown();
    return 0;
}
