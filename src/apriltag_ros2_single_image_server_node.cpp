#include "rclcpp/rclcpp.hpp"
#include "apriltag_ros2/srv/analyze_single_image.hpp"

#include "apriltag_ros2/single_image_detector.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("apriltag_ros2_single_image_server");
  // apriltag_ros2::SingleImageDetector single_tag_detector(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
}