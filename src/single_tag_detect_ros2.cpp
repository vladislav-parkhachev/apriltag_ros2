#include "rclcpp/rclcpp.hpp"
#include <apriltag_ros2/srv/analyze_single_image.hpp>

#include "apriltag_ros2/single_image_detector.h"
#include "apriltag_ros2/common_functions.h"
#include "sensor_msgs/msg/camera_info.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    cv::Mat image = cv::imread("/home/vlad/Documents/apriltag_test.png", cv::IMREAD_COLOR);
    cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image));
    loaded_image->header.frame_id = "camera";
    apriltag_ros2::TagDetector tag_detector_;

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.distortion_model = "plumb_bob";
    camera_info.height = 480;
    camera_info.width = 640;
    camera_info.d = { -0.361976, 0.110510, 0.001014, 0.000505, 0.000000};
    camera_info.k = {438.783367, 0.000000, 305.593336, 0.000000, 437.302876, 243.738352, 0.000000, 0.000000, 1.000000};
    camera_info.r = {0.999978, 0.002789, -0.006046, -0.002816, 0.999986, -0.004401, 0.006034, 0.004417, 0.999972};
    camera_info.p = {393.653800, 0.000000, 322.797939, 0.000000, 0.000000, 393.653800, 241.090902, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};
    
    tag_detector_.detectTags(loaded_image, camera_info);
    tag_detector_.drawDetections(loaded_image);
    cv::imwrite("/home/vlad/Documents/apriltag_detect.png", loaded_image->image);
  
    rclcpp::shutdown();
    return 0;
}