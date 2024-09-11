#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
// #include "apriltag_ros2_interfaces/srv/analyze_single_image.hpp"
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/opencv.hpp>
#include <apriltag.h>
#include "tag36h11.h"

#include <iostream>
#include <thread>

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
  StandaloneTagDescription(int id, double size,
                           std::string &frame_name) :
      id_(id),
      size_(size),
      frame_name_(frame_name) {}

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
  std::map<int, int> id2idx_; // (id2idx_[<tag ID>]=<index in tags_>) mapping

  TagBundleDescription(const std::string& name) :
      name_(name) {}

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
  std::vector<TagBundleMember> tags_;
};

class TagDetector
{
 private:
  // Detections sorting
  // static int idComparison(const void* first, const void* second);

  // // Remove detections of tags with the same ID
  // void removeDuplicates();

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
  std::vector<TagBundleDescription> tag_bundle_descriptions_;
//   bool remove_duplicates_;
  // bool run_quietly_;
  bool publish_tf_;
  // tf::TransformBroadcaster tf_pub_;

 public:

  TagDetector();
  ~TagDetector();

  // // Store standalone and bundle tag descriptions
  // std::map<int, StandaloneTagDescription> parseStandaloneTags(
  //     XmlRpc::XmlRpcValue& standalone_tag_descriptions);
  // std::vector<TagBundleDescription > parseTagBundles(
  //     XmlRpc::XmlRpcValue& tag_bundles);
  // double xmlRpcGetDouble(
  //     XmlRpc::XmlRpcValue& xmlValue, std::string field) const;
  // double xmlRpcGetDoubleWithDefault(
  //     XmlRpc::XmlRpcValue& xmlValue, std::string field,
  //     double defaultValue) const;

  bool findStandaloneTagDescription(
      int id, StandaloneTagDescription*& descriptionContainer,
      bool printWarning = true);

  // geometry_msgs::PoseWithCovarianceStamped makeTagPose(
  //     const Eigen::Isometry3d& transform,
  //     const std_msgs::Header& header);

  // // Detect tags in an image
  // AprilTagDetectionArray detectTags(
  //     const cv_bridge::CvImagePtr& image,
  //     const sensor_msgs::CameraInfoConstPtr& camera_info);

  // // Get the pose of the tag in the camera frame
  // // Returns homogeneous transformation matrix [R,t;[0 0 0 1]] which
  // // takes a point expressed in the tag frame to the same point
  // // expressed in the camera frame. As usual, R is the (passive)
  // // rotation from the tag frame to the camera frame and t is the
  // // vector from the camera frame origin to the tag frame origin,
  // // expressed in the camera frame.
  // Eigen::Isometry3d getRelativeTransform(
  //     const std::vector<cv::Point3d >& objectPoints,
  //     const std::vector<cv::Point2d >& imagePoints,
  //     double fx, double fy, double cx, double cy) const;
  
  // void addImagePoints(apriltag_detection_t *detection,
  //                     std::vector<cv::Point2d >& imagePoints) const;
  // void addObjectPoints(double s, cv::Matx44d T_oi,
  //                      std::vector<cv::Point3d >& objectPoints) const;

  // // Draw the detected tags' outlines and payload values on the image
  void drawDetections(cv_bridge::CvImagePtr image);

  // bool get_publish_tf() const { return publish_tf_; }
};

TagDetector::TagDetector() :
    family_("tag36h11"),
    threads_(0),
    decimate_(1.0),
    blur_(0.0),
    refine_edges_(1),
    debug_(0),
    max_hamming_distance_(2),
    publish_tf_(false)
{
	tf_ = tag36h11_create();

	if (threads_ == 0)
  	{
    	threads_ = std::max(std::thread::hardware_concurrency() - 1U, 1U);
  	}

    // Create the AprilTag 2 detector
    td_ = apriltag_detector_create();
    apriltag_detector_add_family_bits(td_, tf_, max_hamming_distance_);
    td_->quad_decimate = (float)decimate_;
    td_->quad_sigma = (float)blur_;
    td_->nthreads = threads_;
    td_->debug = debug_;
    td_->refine_edges = refine_edges_;
    detections_ = NULL;
}

TagDetector::~TagDetector() {
	// free memory associated with tag detector
	apriltag_detector_destroy(td_);

	// Free memory associated with the array of tag detections
	if(detections_)
	{
		apriltag_detections_destroy(detections_);
	}

	if (family_ == "tag36h11")
	{
		tag36h11_destroy(tf_);
	}
}

bool TagDetector::findStandaloneTagDescription (
    int id, StandaloneTagDescription*& descriptionContainer, bool printWarning)
{
  std::map<int, StandaloneTagDescription>::iterator description_itr =
      standalone_tag_descriptions_.find(id);
  if (description_itr == standalone_tag_descriptions_.end())
  {
    if (printWarning)
    {

    }
    return false;
  }
  descriptionContainer = &(description_itr->second);
  return true;
}

void TagDetector::drawDetections(cv_bridge::CvImagePtr image)
{
	
	cv::Mat gray_image;
	if (image->image.channels() == 1)
	{
		gray_image = image->image;
	}
	else
	{
		cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);
	}
	image_u8_t apriltag_image = { .width = gray_image.cols,
									.height = gray_image.rows,
									.stride = gray_image.cols,
									.buf = gray_image.data


	};

	// Run AprilTag 2 algorithm on the image
	if (detections_)
	{
		apriltag_detections_destroy(detections_);
		detections_ = NULL;
	}
	detections_ = apriltag_detector_detect(td_, &apriltag_image);

	// if (remove_duplicates_)
  	// {
    // 	removeDuplicates();
  	// }


	for (int i = 0; i < zarray_size(detections_); i++)
	{
		apriltag_detection_t *det;
		zarray_get(detections_, i, &det);

		// Check if this ID is present in config/tags.yaml
		// Check if is part of a tag bundle
		int tagID = det->id;
		bool is_part_of_bundle = false;
		for (unsigned int j=0; j<tag_bundle_descriptions_.size(); j++)
		{
		TagBundleDescription bundle = tag_bundle_descriptions_[j];
		if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end())
		{
			is_part_of_bundle = true;
			break;
		}
		}
		// If not part of a bundle, check if defined as a standalone tag
		StandaloneTagDescription* standaloneDescription;
		if (!is_part_of_bundle &&
			!findStandaloneTagDescription(tagID, standaloneDescription, false))
		{
		// Neither a standalone tag nor part of a bundle, so this is a "rogue"
		// tag, skip it.
		continue;
		}

		// Draw tag outline with edge colors green, blue, blue, red
		// (going counter-clockwise, starting from lower-left corner in
		// tag coords). cv::Scalar(Blue, Green, Red) format for the edge
		// colors!
		line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
			cv::Point((int)det->p[1][0], (int)det->p[1][1]),
			cv::Scalar(0, 0xff, 0)); // green
		line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
			cv::Point((int)det->p[3][0], (int)det->p[3][1]),
			cv::Scalar(0, 0, 0xff)); // red
		line(image->image, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
			cv::Point((int)det->p[2][0], (int)det->p[2][1]),
			cv::Scalar(0xff, 0, 0)); // blue
		line(image->image, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
			cv::Point((int)det->p[3][0], (int)det->p[3][1]),
			cv::Scalar(0xff, 0, 0)); // blue

		// Print tag ID in the middle of the tag
		std::stringstream ss;
		ss << det->id;
		cv::String text = ss.str();
		int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
		double fontscale = 0.5;
		int baseline;
		cv::Size textsize = cv::getTextSize(text, fontface,
											fontscale, 2, &baseline);
		cv::putText(image->image, text,
					cv::Point((int)(det->c[0]-textsize.width/2),
							(int)(det->c[1]+textsize.height/2)),
					fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
	}
}


void apriltag_ros2_single_image_detect()
{
    cv::Mat image = cv::imread("/home/vlad/Documents/apriltag_test.png", cv::IMREAD_COLOR);
    cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image));
    loaded_image->header.frame_id = "camera";
	TagDetector tag_detector_;
    tag_detector_.drawDetections(loaded_image);
    cv::imwrite("/home/vlad/Documents/apriltag_detect.png", loaded_image->image);
	// cv::imwrite("/home/vlad/Documents/apriltag_detect.png", image);
}

int main(int argc, char **argv)
{
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("apriltag_ros2_single_image_server_node"); 
  apriltag_ros2_single_image_detect();
//   rclcpp::shutdown();
}