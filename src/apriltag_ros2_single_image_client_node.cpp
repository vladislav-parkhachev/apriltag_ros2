#include "rclcpp/rclcpp.hpp"
#include "apriltag_ros2/srv/analyze_single_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("apriltag_ros_single_image_client");
    auto client = node->create_client<apriltag_ros2::srv::AnalyzeSingleImage>("single_image_tag_detection");
    auto request = std::make_shared<apriltag_ros2::srv::AnalyzeSingleImage::Request>();
    request->full_path_where_to_get_image = "/home/vlad/Documents/apriltag_test.png";
    request->full_path_where_to_save_image = "/home/vlad/Documents/apriltag_ros2.png";
    request->camera_info.distortion_model = "plumb_bob";
    request->camera_info.height = 480;
    request->camera_info.width = 640;
    request->camera_info.d = { -0.361976, 0.110510, 0.001014, 0.000505, 0.000000};
    request->camera_info.k = {438.783367, 0.000000, 305.593336, 0.000000, 437.302876, 243.738352, 0.000000, 0.000000, 1.000000};
    request->camera_info.r = {0.999978, 0.002789, -0.006046, -0.002816, 0.999986, -0.004401, 0.006034, 0.004417, 0.999972};
    request->camera_info.p = {393.653800, 0.000000, 322.797939, 0.000000, 0.000000, 393.653800, 241.090902, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};

    rclcpp::shutdown();
    return 0;
}