#include "x4_vision/aruco_detector.hpp"

ArucoDetector::ArucoDetector() : Node("aruco_detector"), camera_info_ready_(false) {
  this->declare_parameter<double>("marker_size", 0.15);
  this->get_parameter("marker_size", marker_size_);

  // load ArUco dictionary
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  
  // set up subscriptions
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1)); // raw camera feed

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", rclcpp::QoS(10).transient_local().reliable(),
    std::bind(&ArucoDetector::camera_info_callback, this, std::placeholders::_1)); // camera intrinsics

  // publishers - publish bounding box detections & 3D poses for each marker
  detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
    "/aruco/detections", 10);

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/aruco/pose", 10);
}

void ArucoDetector::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
 // this is called once when camera calibration is received
  if (camera_info_ready_)
    return;
  
  // load intrinsic K matrix and distortion coeffs
  camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
  dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();

  camera_info_ready_ = true;
  RCLCPP_INFO(this->get_logger(), "Camera info received and matrix stored");
}

void ArucoDetector::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
  // check calibration - don't process images until we've received intrinsics
  if (!camera_info_ready_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for camera info...");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); // converts ROS 2 image message to OpenCV format
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids); // detect ArUco Markers

  // publish 2D Bounding Boxes 
  vision_msgs::msg::Detection2DArray detections;
  detections.header = msg->header;

  if (!ids.empty()) {
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs); // pose estimation - rvecs: rotation vectors, tvecs: translation vectors

    for (size_t i = 0; i < ids.size(); ++i) {
      // Populate 2D detection for each detected marker
      vision_msgs::msg::Detection2D det;
      det.results.resize(1);
      det.results[0].hypothesis.class_id = std::to_string(ids[i]); // use class_id instead of id
      det.results[0].hypothesis.score = 1.0;

      det.bbox.center.position.x = (corners[i][0].x + corners[i][2].x) / 2.0;
      det.bbox.center.position.y = (corners[i][0].y + corners[i][2].y) / 2.0;
      det.bbox.size_x = std::abs(corners[i][0].x - corners[i][2].x);
      det.bbox.size_y = std::abs(corners[i][0].y - corners[i][2].y);
      detections.detections.push_back(det);

      // Populate 3D pose
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = msg->header;

      cv::Vec3d rvec = rvecs[i];
      cv::Vec3d tvec = tvecs[i];

      cv::Mat R;
      cv::Rodrigues(rvec, R); // convert rotation to quaternion

      // Convert rotation matrix to quaternion
      double qw = std::sqrt(1.0 + R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2)) / 2.0;
      double qx = (R.at<double>(2,1) - R.at<double>(1,2)) / (4.0 * qw);
      double qy = (R.at<double>(0,2) - R.at<double>(2,0)) / (4.0 * qw);
      double qz = (R.at<double>(1,0) - R.at<double>(0,1)) / (4.0 * qw);

      pose_msg.pose.position.x = tvec[0];
      pose_msg.pose.position.y = tvec[1];
      pose_msg.pose.position.z = tvec[2];
      pose_msg.pose.orientation.x = qx;
      pose_msg.pose.orientation.y = qy;
      pose_msg.pose.orientation.z = qz;
      pose_msg.pose.orientation.w = qw;

      pose_pub_->publish(pose_msg); // publish the 3D pose of each marker
    }
  }

  detection_pub_->publish(detections);
}


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetector>());
  rclcpp::shutdown();
  return 0;
}

