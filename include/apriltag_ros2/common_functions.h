#ifndef APRILTAG_ROS_COMMON_FUNCTIONS_H
#define APRILTAG_ROS_COMMON_FUNCTIONS_H

#include "rclcpp/rclcpp.hpp"

#include "apriltag_ros2/msg/april_tag_detection.hpp"
#include "apriltag_ros2/msg/april_tag_detection_array.hpp"

#include <apriltag.h>


namespace apriltag_ros2
{

class StandaloneTagDescription
{
public:
    StandaloneTagDescription() {};
    StandaloneTagDescription(int id, double size, std::string &frame_name) : id_(id),size_(size), frame_name_(frame_name) {}

    double size() { return size_; }
    int id() { return id_; }
    std::string& frame_name() { return frame_name_; }

 private:
    // Tag description
    int id_;
    double size_;
    std::string frame_name_;
};

}


#endif // APRILTAG_ROS_COMMON_FUNCTIONS_H