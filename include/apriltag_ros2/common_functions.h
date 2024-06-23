#ifndef APRILTAG_ROS_COMMON_FUNCTIONS_H
#define APRILTAG_ROS_COMMON_FUNCTIONS_H

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <opencv2/opencv.hpp>

#include "apriltag_ros2/msg/april_tag_detection.hpp"
#include "apriltag_ros2/msg/april_tag_detection_array.hpp"

#include <apriltag.h>


namespace apriltag_ros2
{

struct TagBundleMember
{
    int id; // Payload ID
    double size; // [m] Side length
    cv::Matx44d T_oi; // Rigid transform from tag i frame to bundle origin frame
};

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

class TagBundleDescription
{
public:

    TagBundleDescription(const std::string& name) : name_(name) {}

    std::map<int, int > id2idx_; // (id2idx_[<tag ID>]=<index in tags_>) mapping
    
    void addMemberTag(int id, double size, cv::Matx44d T_oi) {
        TagBundleMember member;
        member.id = id;
        member.size = size;
        member.T_oi = T_oi;
        tags_.push_back(member);
        id2idx_[id] = tags_.size()-1;
    }

    const std::string& name() const { return name_; }

    // Get IDs of bundle member tags
    std::vector<int> bundleIds () {
        std::vector<int> ids;
        for (unsigned int i = 0; i < tags_.size(); i++) {
            ids.push_back(tags_[i].id);
        }
        return ids;
    }

    // Get sizes of bundle member tags
    std::vector<double> bundleSizes () {
        std::vector<double> sizes;
        for (unsigned int i = 0; i < tags_.size(); i++) {
            sizes.push_back(tags_[i].size);
        }
        return sizes;
    }

    int memberID (int tagID) { return tags_[id2idx_[tagID]].id; }
    double memberSize (int tagID) { return tags_[id2idx_[tagID]].size; }
    cv::Matx44d memberT_oi (int tagID) { return tags_[id2idx_[tagID]].T_oi; }

 private:
  // Bundle description
  std::string name_;
  std::vector<TagBundleMember > tags_;
};



}


#endif // APRILTAG_ROS_COMMON_FUNCTIONS_H