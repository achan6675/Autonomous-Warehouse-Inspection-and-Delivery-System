#include "turtlebot3_gazebo/qr_detector.hpp"

#include <cmath>
#include <eigen3/Eigen/Geometry>

using namespace std::chrono_literals;

namespace zbar_ros {

// Constructor initializes the QR detector node with ZBar scanner configuration,
// sets up subscriptions to camera topics, and creates publishers for detected
// symbols and poses.
QRDetector::QRDetector() : Node("qr_detector_node") {
  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&QRDetector::imageCb, this, std::placeholders::_1));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10,
      std::bind(&QRDetector::cameraInfoCb, this, std::placeholders::_1));

  symbol_pub_ =
      this->create_publisher<zbar_ros_interfaces::msg::Symbol>("symbol", 10);
  pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("qr_pose", 10);

  barcode_pub_ = this->create_publisher<std_msgs::msg::String>("barcode", 10);

  throttle_ =
      this->declare_parameter<double>("throttle_repeated_barcodes", 0.0);
  RCLCPP_INFO(get_logger(), "throttle_repeated_barcodes : %f", throttle_);

  qr_size_in_m_ = this->declare_parameter<double>("qr_size_in_m", 1.5);

  if (throttle_ > 0.0) {
    clean_timer_ =
        this->create_wall_timer(10s, std::bind(&QRDetector::cleanCb, this));
  }
}

// Processes camera calibration info including intrinsic matrix K and distortion
// coefficients. Stores camera frame ID and sets caminfo_ready_ flag when valid
// data received.
void QRDetector::cameraInfoCb(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
  K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
  dist_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data()))
              .clone();

  if (K_.empty() || dist_.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Camera matrix or distortion coefficients are empty!");
    return;
  }

  camera_frame_id_ = msg->header.frame_id;
  caminfo_ready_ = true;
  // RCLCPP_INFO(this->get_logger(), "Camera info received and processed.");
}

// Main image processing callback. Converts image to grayscale, scans for QR
// codes using ZBar, filters duplicates with throttling, and computes 3D pose
// using solvePnP if camera info available.
void QRDetector::imageCb(sensor_msgs::msg::Image::ConstSharedPtr image) {
  // RCLCPP_INFO(get_logger(), "Image received on subscribed topic");

  cv_bridge::CvImageConstPtr cv_image;
  cv_image = cv_bridge::toCvShare(image, "mono8");

  zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800",
                         cv_image->image.data,
                         cv_image->image.cols * cv_image->image.rows);
  scanner_.scan(zbar_image);

  auto start = zbar_image.symbol_begin();
  auto end = zbar_image.symbol_end();
  if (start != end) {
    for (zbar::Image::SymbolIterator symbol_it = start; symbol_it != end;
         ++symbol_it) {
      zbar_ros_interfaces::msg::Symbol symbol;
      symbol.data = symbol_it->get_data();
      RCLCPP_INFO(get_logger(), "QRcode detected with data: '%s'",
                  symbol.data.c_str());

      const int npts = symbol_it->get_location_size();
      for (int i = 0; i < npts; ++i) {
        vision_msgs::msg::Point2D point;
        point.x = symbol_it->get_location_x(i);
        point.y = symbol_it->get_location_y(i);
        RCLCPP_DEBUG(get_logger(), "  Point: %f, %f", point.x, point.y);
        symbol.points.push_back(point);
      }
      // verify if repeated barcode throttling is enabled
      if (throttle_ > 0.0) {
        const std::lock_guard<std::mutex> lock(memory_mutex_);

        const std::string& barcode = symbol.data;
        // check if barcode has been recorded as seen, and skip detection
        if (barcode_memory_.count(barcode) > 0) {
          // check if time reached to forget barcode
          if (now() > barcode_memory_.at(barcode)) {
            RCLCPP_INFO(get_logger(),
                        "Memory timed out for barcode, publishing");
            barcode_memory_.erase(barcode);
          } else {
            // if timeout not reached, skip this reading
            continue;
          }
        }
        // record barcode as seen, with a timeout to 'forget'
        barcode_memory_.insert(std::make_pair(
            barcode, now() + rclcpp::Duration(
                                 std::chrono::duration<double>(throttle_))));
      }
      symbol_pub_->publish(symbol);

      std_msgs::msg::String barcode_string;
      barcode_string.data = symbol.data;
      barcode_pub_->publish(barcode_string);

      if (!caminfo_ready_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                             "CameraInfo not received yet; skip PnP.");
      } else {
        std::vector<cv::Point2f> poly_px;
        poly_px.reserve(symbol.points.size());
        for (auto& p : symbol.points) poly_px.emplace_back(p.x, p.y);

        std::vector<cv::Point2f> imgPts4;
        if (!fourCornersFromPolygon(poly_px, imgPts4)) {
          RCLCPP_DEBUG(get_logger(),
                       "Could not derive 4 corners from polygon; skip PnP");
        } else {
          sortCornersClockwise(imgPts4);

          const float S = static_cast<float>(qr_size_in_m_);
          std::vector<cv::Point3f> objPts = {{-S * 0.5f, -S * 0.5f, 0.f},
                                             {S * 0.5f, -S * 0.5f, 0.f},
                                             {S * 0.5f, S * 0.5f, 0.f},
                                             {-S * 0.5f, S * 0.5f, 0.f}};

          cv::Mat rvec, tvec;
          bool ok = false;
          ok = cv::solvePnP(objPts, imgPts4, K_, dist_, rvec, tvec, false,
                            cv ::SOLVEPNP_IPPE_SQUARE);
          if (!ok) {
            RCLCPP_DEBUG(get_logger(), "solvePnP failed");
          } else {
            cv::Mat R;
            cv::Rodrigues(rvec, R);

            Eigen::Matrix3d Re;
            for (int r = 0; r < 3; ++r)
              for (int c = 0; c < 3; ++c) Re(r, c) = R.at<double>(r, c);
            Eigen::Quaterniond q_opt(Re);

            const double x_cv = tvec.at<double>(0);
            const double y_cv = tvec.at<double>(1);
            const double z_cv = tvec.at<double>(2);

            Eigen::Matrix3d R_opt_to_link;
            R_opt_to_link << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
            Eigen::Quaterniond q_fix(R_opt_to_link);
            Eigen::Quaterniond q_link = q_fix * q_opt;

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id =
                camera_frame_id_.empty() ? "camera_frame" : camera_frame_id_;

            pose.pose.position.x = z_cv;
            pose.pose.position.y = -x_cv;
            pose.pose.position.z = -y_cv;
            pose.pose.orientation.x = q_link.x();
            pose.pose.orientation.y = q_link.y();
            pose.pose.orientation.z = q_link.z();
            pose.pose.orientation.w = q_link.w();

            pose_pub_->publish(pose);

            const double dist =
                std::sqrt(x_cv * x_cv + y_cv * y_cv + z_cv * z_cv);
            const double x_lateral = x_cv;
            const double y_vertical = y_cv;
            const double z_depth = z_cv;

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "QR Pose - X:%.3fm, Y:%.3fm, "
                                 "Z:%.3fm, distance:%.3fm",
                                 x_lateral, y_vertical, z_depth, dist);
          }
        }
      }
    }
  } else {
    RCLCPP_INFO(get_logger(), "No barcode detected in image");
  }
  static bool alreadyWarnedDeprecation = false;
  if (!alreadyWarnedDeprecation && count_subscribers("barcode") > 0) {
    alreadyWarnedDeprecation = true;
  }

  zbar_image.set_data(NULL, 0);
}

// Sorts four corner points in clockwise order starting from top-left corner.
// Used to ensure consistent corner ordering for solvePnP pose estimation.
void QRDetector::sortCornersClockwise(std::vector<cv::Point2f>& pts) {
  cv::Point2f c(0, 0);
  for (auto& p : pts) c += p;
  c *= (1.0f / pts.size());
  std::sort(pts.begin(), pts.end(),
            [c](const cv::Point2f& a, const cv::Point2f& b) {
              return std::atan2(a.y - c.y, a.x - c.x) <
                     std::atan2(b.y - c.y, b.x - c.x);
            });
  int idx0 = 0;
  float best = 1e9f;
  for (int i = 0; i < 4; ++i) {
    float score = pts[i].x + pts[i].y;
    if (score < best) {
      best = score;
      idx0 = i;
    }
  }
  std::rotate(pts.begin(), pts.begin() + idx0, pts.end());
}

// Extracts or approximates four corners from a polygon of QR code boundary
// points. Returns true if successful; uses convex hull and minimum area
// rectangle if needed.
bool QRDetector::fourCornersFromPolygon(const std::vector<cv::Point2f>& poly,
                                        std::vector<cv::Point2f>& out4) {
  if (poly.size() == 4) {
    out4 = poly;
    return true;
  }
  if (poly.size() < 4) return false;

  std::vector<cv::Point2f> hull;
  cv::convexHull(poly, hull);
  if (hull.size() < 4) return false;

  cv::RotatedRect box = cv::minAreaRect(hull);
  cv::Point2f rect_pts[4];
  box.points(rect_pts);
  out4.assign(rect_pts, rect_pts + 4);
  return true;
}

// Timer callback to clean expired barcode entries from memory cache. Removes
// barcodes whose throttle timeout has expired to allow re-detection.
void QRDetector::cleanCb() {
  const std::lock_guard<std::mutex> lock(memory_mutex_);
  auto it = barcode_memory_.begin();
  while (it != barcode_memory_.end()) {
    if (now() > it->second) {
      RCLCPP_INFO(get_logger(), "Cleaned %s from memory", it->first.c_str());
      it = barcode_memory_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace zbar_ros
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zbar_ros::QRDetector>());
  rclcpp::shutdown();
  return 0;
}