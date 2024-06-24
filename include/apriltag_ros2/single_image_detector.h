#include "apriltag_ros2/common_functions.h"
#include <apriltag_ros2/srv/analyze_single_image.hpp>

#ifndef APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H
#define APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H

namespace apriltag_ros2
{

class SingleImageDetector
{
private:
    TagDetector tag_detector_;
    rclcpp::Service<apriltag_ros2::srv::AnalyzeSingleImage>::SharedPtr single_image_analysis_service_;
    rclcpp::Publisher<apriltag_ros2::msg::AprilTagDetectionArray>::SharedPtr tag_detections_publisher_;
public:
    SingleImageDetector(std::shared_ptr<rclcpp::Node> node);

    // The function which provides the single image analysis service
    bool analyzeImage(apriltag_ros2::srv::AnalyzeSingleImage::Request& request, apriltag_ros2::srv::AnalyzeSingleImage::Response& response);
};

}

#endif // APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H