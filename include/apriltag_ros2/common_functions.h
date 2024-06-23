#ifndef APRILTAG_ROS_COMMON_FUNCTIONS_H
#define APRILTAG_ROS_COMMON_FUNCTIONS_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
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

class TagDetector
{
private:
    // Detections sorting
    static int idComparison(const void* first, const void* second);

    // Remove detections of tags with the same ID
    void removeDuplicates();

    // AprilTag 2 code's attributes
    std::string family_;
    int threads_;
    double decimate_;
    double blur_;
    int refine_edges_;
    int debug_;
    int max_hamming_distance_ = 2;  // Tunable, but really, 2 is a good choice. Values of >=3
                                    // consume prohibitively large amounts of memory, and otherwise
                                    // you want the largest value possible.

    // AprilTag 2 objects
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    zarray_t *detections_;

    // Other members
    std::map<int, StandaloneTagDescription> standalone_tag_descriptions_;
    std::vector<TagBundleDescription > tag_bundle_descriptions_;
    bool remove_duplicates_;
    bool run_quietly_;
    bool publish_tf_;
    tf2_ros::TransformBroadcaster tf_pub_;

 public:

    TagDetector(std::shared_ptr<rclcpp::Node> node);
    ~TagDetector();

    // Store standalone and bundle tag descriptions
    // std::map<int, StandaloneTagDescription> parseStandaloneTags(XmlRpc::XmlRpcValue& standalone_tag_descriptions);
    // std::vector<TagBundleDescription> parseTagBundles(XmlRpc::XmlRpcValue& tag_bundles);
    // double xmlRpcGetDouble(XmlRpc::XmlRpcValue& xmlValue, std::string field) const;
    // double xmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue& xmlValue, std::string field, double defaultValue) const;

    bool findStandaloneTagDescription(int id, StandaloneTagDescription*& descriptionContainer, bool printWarning = true);

    geometry_msgs::msg::PoseWithCovarianceStamped makeTagPose(const Eigen::Isometry3d& transform, const std_msgs::msg::Header& header);

    // Detect tags in an image
    apriltag_ros2::msg::AprilTagDetectionArray detectTags(const cv_bridge::CvImagePtr& image, const sensor_msgs::msg::CameraInfo& camera_info);

    // Get the pose of the tag in the camera frame
    // Returns homogeneous transformation matrix [R,t;[0 0 0 1]] which
    // takes a point expressed in the tag frame to the same point
    // expressed in the camera frame. As usual, R is the (passive)
    // rotation from the tag frame to the camera frame and t is the
    // vector from the camera frame origin to the tag frame origin,
    // expressed in the camera frame.
    Eigen::Isometry3d getRelativeTransform(
        const std::vector<cv::Point3d >& objectPoints,
        const std::vector<cv::Point2d >& imagePoints,
        double fx, double fy, double cx, double cy) const;
    
    void addImagePoints(apriltag_detection_t *detection, std::vector<cv::Point2d >& imagePoints) const;
    void addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d >& objectPoints) const;

    // Draw the detected tags' outlines and payload values on the image
    void drawDetections(cv_bridge::CvImagePtr image);

    bool get_publish_tf() const { return publish_tf_; }
    };

}

#endif // APRILTAG_ROS_COMMON_FUNCTIONS_H