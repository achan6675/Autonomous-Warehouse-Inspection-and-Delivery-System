#include "turtlebot3_gazebo/qr_transform.hpp"

// Constructor initializes static TF broadcaster, TF buffer/listener, dynamic
// broadcaster, and sets up subscriptions for QR pose transformation from camera
// to map frame.
TFStaticBroadcaster::TFStaticBroadcaster()
    : Node("tf_static_broadcaster_node") {
  static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  publish_static_base_to_camera();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  dyn_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  qr_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "qr_pose", 10,
      std::bind(&TFStaticBroadcaster::qrPoseCb, this, std::placeholders::_1));
  qr_in_map_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "qr_pose_in_map", 10);
  qr_xy_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "qr_xy_in_map", 10);
}

// Publishes static transform from base_link to camera_frame with fixed
// translation and rotation parameters. Called once during node initialization.
void TFStaticBroadcaster::publish_static_base_to_camera() {
  const std::string parent_frame = "base_link";
  const std::string child_frame = "camera_frame";
  const double tx = 0.10;    // m
  const double ty = 0.0;     // m
  const double tz = 0.115;   // m
  const double roll = 0.0;   // rad
  const double pitch = 0.0;  // rad
  const double yaw = 1.57;   // rad

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = parent_frame;
  t.child_frame_id = child_frame;
  t.transform.translation.x = tx;
  t.transform.translation.y = ty;
  t.transform.translation.z = tz;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  static_broadcaster_->sendTransform(t);

  RCLCPP_INFO(get_logger(),
              "Static TF published: %s -> %s | t[%.3f, %.3f, %.3f] m, "
              "rpy[%.3f, %.3f, %.3f] rad",
              parent_frame.c_str(), child_frame.c_str(), tx, ty, tz, roll,
              pitch, yaw);
}

// Callback for QR pose messages in camera frame. Transforms pose to map frame
// using TF, publishes full pose and XY point, and broadcasts dynamic TF for
// visualization.
void TFStaticBroadcaster::qrPoseCb(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const std::string source_frame = "camera_frame";
  const std::string target_frame = "map";

  try {
    if (!tf_buffer_->canTransform(target_frame, source_frame, msg->header.stamp,
                                  std::chrono::milliseconds(200))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF not available: %s <- %s at %.3f s",
                           target_frame.c_str(), source_frame.c_str(),
                           rclcpp::Time(msg->header.stamp).seconds());
      return;
    }

    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
        target_frame, source_frame, msg->header.stamp,
        std::chrono::milliseconds(200));

    geometry_msgs::msg::PoseStamped out_in_map;
    tf2::doTransform(*msg, out_in_map, tf);
    out_in_map.header.stamp = msg->header.stamp;
    out_in_map.header.frame_id = target_frame;

    qr_in_map_pub_->publish(out_in_map);

    {
      geometry_msgs::msg::PointStamped pt;
      pt.header = out_in_map.header;
      pt.point.x = out_in_map.pose.position.x;
      pt.point.y = out_in_map.pose.position.y;
      pt.point.z = 0.0;
      qr_xy_pub_->publish(pt);
    }

    {
      geometry_msgs::msg::TransformStamped cam_to_qr;
      cam_to_qr.header.stamp = msg->header.stamp;
      cam_to_qr.header.frame_id = source_frame;
      cam_to_qr.child_frame_id = "qr_code_frame";
      cam_to_qr.transform.translation.x = msg->pose.position.x;
      cam_to_qr.transform.translation.y = msg->pose.position.y;
      cam_to_qr.transform.translation.z = msg->pose.position.z;
      cam_to_qr.transform.rotation = msg->pose.orientation;
      dyn_broadcaster_->sendTransform(cam_to_qr);
    }

    {
      geometry_msgs::msg::TransformStamped map_to_qr;
      map_to_qr.header.stamp = out_in_map.header.stamp;
      map_to_qr.header.frame_id = target_frame;
      map_to_qr.child_frame_id = "qr_code_map";
      map_to_qr.transform.translation.x = out_in_map.pose.position.x;
      map_to_qr.transform.translation.y = out_in_map.pose.position.y;
      map_to_qr.transform.translation.z = out_in_map.pose.position.z;
      map_to_qr.transform.rotation = out_in_map.pose.orientation;
      dyn_broadcaster_->sendTransform(map_to_qr);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "QR in %s: [x=%.3f, y=%.3f, z=%.3f] (from %s)",
                         target_frame.c_str(), out_in_map.pose.position.x,
                         out_in_map.pose.position.y, out_in_map.pose.position.z,
                         source_frame.c_str());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Transform failed %s <- %s: %s", target_frame.c_str(),
                         source_frame.c_str(), ex.what());
  }
}

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFStaticBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
