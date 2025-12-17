#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector : public rclcpp::Node {
public:
  ArucoDetector();

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg); // gets called when image is received
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  bool camera_info_ready_;


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; // subscriber to /camera/image_raw
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_; // publisher to /aruco/detections
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_; // OpenCV ArUco marker dictionary 
  double marker_size_; // meters - used for pose estimation
};

