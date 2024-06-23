#include "apriltag_ros2/single_image_detector.h"

#include "std_msgs/msg/header.hpp"

namespace apriltag_ros2
{
SingleImageDetector::SingleImageDetector (std::shared_ptr<rclcpp::Node> node) : tag_detector_(node)
{
    // Advertise the single image analysis service    
    single_image_analysis_service_ = node->create_service<apriltag_ros2::srv::AnalyzeSingleImage>("single_image_tag_detection", this);
    tag_detections_publisher_ = node->create_publisher<apriltag_ros2::msg::AprilTagDetectionArray>("tag_detections", 1);
}

bool SingleImageDetector::analyzeImage(apriltag_ros2::srv::AnalyzeSingleImage::Request& request, apriltag_ros2::srv::AnalyzeSingleImage::Response& response)
{
    // Read the image
    cv::Mat image = cv::imread(request.full_path_where_to_get_image, cv::IMREAD_COLOR);

    if (image.data == NULL)
    {
        return false;
    }

    // Detect tags in the image
    cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image));

    loaded_image->header.frame_id = "camera";
    
    response.tag_detections = tag_detector_.detectTags(loaded_image, request.camera_info);

    // Publish detected tags (AprilTagDetectionArray, basically an array of geometry_msgs/PoseWithCovarianceStamped)
    tag_detections_publisher_->publish(response.tag_detections);

    // Save tag detections image
    tag_detector_.drawDetections(loaded_image);
    cv::imwrite(request.full_path_where_to_save_image, loaded_image->image);

    return true;
}

}