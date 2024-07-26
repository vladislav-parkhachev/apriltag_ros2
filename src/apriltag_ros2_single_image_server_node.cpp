#include "rclcpp/rclcpp.hpp"
#include "apriltag_ros2/srv/analyze_single_image.hpp"

#include "apriltag_ros2/single_image_detector.h"

void tag_detect(const std::shared_ptr<apriltag_ros2::srv::AnalyzeSingleImage::Request> request)
{
    // apriltag_ros2::SingleImageDetector 
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("apriltag_ros2_single_image_server");
  auto service = node->create_service<apriltag_ros2::srv::AnalyzeSingleImage>("tag_detect", &tag_detect);
  // apriltag_ros2::SingleImageDetector single_tag_detector(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
}