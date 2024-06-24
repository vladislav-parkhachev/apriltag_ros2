#include "apriltag_ros2/common_functions.h"
#include "image_geometry/pinhole_camera_model.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tagStandard52h13.h"
#include "tagStandard41h12.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCustom48h12.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"


namespace apriltag_ros2
{
    TagDetector::TagDetector(std::shared_ptr<rclcpp::Node> node) :
    family_(getAprilTagOption<std::string>(node, "tag_family", "tag36h11")),
    threads_(getAprilTagOption<int>(node, "tag_threads", 0)),
    decimate_(getAprilTagOption<double>(node, "tag_decimate", 1.0)),
    blur_(getAprilTagOption<double>(node, "tag_blur", 0.0)),
    refine_edges_(getAprilTagOption<int>(node, "tag_refine_edges", 1)),
    debug_(getAprilTagOption<int>(node, "tag_debug", 0)),
    max_hamming_distance_(getAprilTagOption<int>(node, "max_hamming_dist", 2)),
    publish_tf_(getAprilTagOption<bool>(node, "publish_tf", false))
    {
        if (family_ == "tagStandard52h13")      { tf_ = tagStandard52h13_create(); }
        else if (family_ == "tagStandard41h12") { tf_ = tagStandard41h12_create(); }
        else if (family_ == "tag36h11")         { tf_ = tag36h11_create();}
        else if (family_ == "tag25h9")          { tf_ = tag25h9_create(); }
        else if (family_ == "tag16h5")          { tf_ = tag16h5_create(); }
        else if (family_ == "tagCustom48h12")   { tf_ = tagCustom48h12_create(); }
        else if (family_ == "tagCircle21h7")    { tf_ = tagCircle21h7_create(); }
        else if (family_ == "tagCircle49h12")   { tf_ = tagCircle49h12_create(); }
        else                                    { exit(1);}

        if (threads_ == 0){
            threads_ = std::max(std::thread::hardware_concurrency() - 1U, 1U);
            // ROS_INFO("Thread count not specified. Using %d threads", threads_);
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

    // destructor
    TagDetector::~TagDetector() {

        // free memory associated with tag detector
        apriltag_detector_destroy(td_);

        // Free memory associated with the array of tag detections
        if(detections_){ apriltag_detections_destroy(detections_); }

        // free memory associated with tag family
        if (family_ == "tagStandard52h13")      { tagStandard52h13_destroy(tf_);}
        else if (family_ == "tagStandard41h12") { tagStandard41h12_destroy(tf_); }
        else if (family_ == "tag36h11")         { tag36h11_destroy(tf_); }
        else if (family_ == "tag25h9")          { tag25h9_destroy(tf_); }
        else if (family_ == "tag16h5")          { tag16h5_destroy(tf_); }
        else if (family_ == "tagCustom48h12")   { tagCustom48h12_destroy(tf_); }
        else if (family_ == "tagCircle21h7")    { tagCircle21h7_destroy(tf_); }
        else if (family_ == "tagCircle49h12")   { tagCircle49h12_destroy(tf_); }

    }

    apriltag_ros2::msg::AprilTagDetectionArray TagDetector::detectTags(
        const cv_bridge::CvImagePtr& image, 
        const sensor_msgs::msg::CameraInfo& camera_info) {

            // Convert image to AprilTag code's format
            cv::Mat gray_image;
            if (image->image.channels() == 1){
                gray_image = image->image;
            }
            else{
                cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);
            }

            image_u8_t apriltag_image = { .width = gray_image.cols,
                                            .height = gray_image.rows,
                                            .stride = gray_image.cols,
                                            .buf = gray_image.data };


            image_geometry::PinholeCameraModel camera_model;
            camera_model.fromCameraInfo(camera_info);

            // Get camera intrinsic properties for rectified image.
            double fx = camera_model.fx(); // focal length in camera x-direction [px]
            double fy = camera_model.fy(); // focal length in camera y-direction [px]
            double cx = camera_model.cx(); // optical center x-coordinate [px]
            double cy = camera_model.cy(); // optical center y-coordinate [px]

            // Check if camera intrinsics are not available - if not the calculated
            // transforms are meaningless.
            //if (fx == 0 && fy == 0) ROS_WARN_STREAM_THROTTLE(5, "fx and fy are zero. Are the camera intrinsics set?");

            // Run AprilTag 2 algorithm on the image
            if (detections_)
            {
                apriltag_detections_destroy(detections_);
                detections_ = NULL;
            }
            detections_ = apriltag_detector_detect(td_, &apriltag_image);

            // If remove_duplicates_ is set to true, then duplicate tags are not allowed.
            // Thus any duplicate tag IDs visible in the scene must include at least 1
            // erroneous detection. Remove any tags with duplicate IDs to ensure removal
            // of these erroneous detections
            if (remove_duplicates_)
            {
                removeDuplicates();
            }

            // Compute the estimated translation and rotation individually for each
            // detected tag
            apriltag_ros2::msg::AprilTagDetectionArray tag_detection_array;
            std::vector<std::string > detection_names;
            tag_detection_array.header = image->header;
            std::map<std::string, std::vector<cv::Point3d > > bundleObjectPoints;
            std::map<std::string, std::vector<cv::Point2d > > bundleImagePoints;

            for (int i=0; i < zarray_size(detections_); i++)
            {
                // Get the i-th detected tag
                apriltag_detection_t *detection;
                zarray_get(detections_, i, &detection);

                // Bootstrap this for loop to find this tag's description amongst
                // the tag bundles. If found, add its points to the bundle's set of
                // object-image corresponding points (tag corners) for cv::solvePnP.
                // Don't yet run cv::solvePnP on the bundles, though, since we're still in
                // the process of collecting all the object-image corresponding points
                int tagID = detection->id;
                bool is_part_of_bundle = false;

                for (unsigned int j=0; j<tag_bundle_descriptions_.size(); j++)
                {
                    // Iterate over the registered bundles
                    TagBundleDescription bundle = tag_bundle_descriptions_[j];


                    if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end())
                    {
                        // This detected tag belongs to the j-th tag bundle (its ID was found in
                        // the bundle description)
                        is_part_of_bundle = true;
                        std::string bundleName = bundle.name();

                        //===== Corner points in the world frame coordinates
                        double s = bundle.memberSize(tagID)/2;
                        addObjectPoints(s, bundle.memberT_oi(tagID),
                                        bundleObjectPoints[bundleName]);

                        //===== Corner points in the image frame coordinates
                        addImagePoints(detection, bundleImagePoints[bundleName]);
                    }
                }

                // Find this tag's description amongst the standalone tags
                // Print warning when a tag was found that is neither part of a
                // bundle nor standalone (thus it is a tag in the environment
                // which the user specified no description for, or Apriltags
                // misdetected a tag (bad ID or a false positive)).
                StandaloneTagDescription* standaloneDescription;
                if (!findStandaloneTagDescription(tagID, standaloneDescription, !is_part_of_bundle))
                {
                    continue;
                }

                //=================================================================
                // The remainder of this for loop is concerned with standalone tag
                // poses!
                double tag_size = standaloneDescription->size();


                // Get estimated tag pose in the camera frame.
                //
                // Note on frames:
                // The raw AprilTag 2 uses the following frames:
                //   - camera frame: looking from behind the camera (like a
                //     photographer), x is right, y is up and z is towards you
                //     (i.e. the back of camera)
                //   - tag frame: looking straight at the tag (oriented correctly),
                //     x is right, y is down and z is away from you (into the tag).
                // But we want:
                //   - camera frame: looking from behind the camera (like a
                //     photographer), x is right, y is down and z is straight
                //     ahead
                //   - tag frame: looking straight at the tag (oriented correctly),
                //     x is right, y is up and z is towards you (out of the tag).
                // Using these frames together with cv::solvePnP directly avoids
                // AprilTag 2's frames altogether.
                // TODO solvePnP[Ransac] better?
                std::vector<cv::Point3d > standaloneTagObjectPoints;
                std::vector<cv::Point2d > standaloneTagImagePoints;
                addObjectPoints(tag_size/2, cv::Matx44d::eye(), standaloneTagObjectPoints);
                addImagePoints(detection, standaloneTagImagePoints);

                Eigen::Isometry3d transform = getRelativeTransform(standaloneTagObjectPoints, standaloneTagImagePoints, fx, fy, cx, cy);

                geometry_msgs::msg::PoseWithCovarianceStamped tag_pose = makeTagPose(transform, image->header);

                    // Add the detection to the back of the tag detection array
                apriltag_ros2::msg::AprilTagDetection tag_detection;
                tag_detection.pose = tag_pose;
                tag_detection.id.push_back(detection->id);
                tag_detection.size.push_back(tag_size);
                tag_detection_array.detections.push_back(tag_detection);
                detection_names.push_back(standaloneDescription->frame_name());
            }

            //=================================================================
            // Estimate bundle origin pose for each bundle in which at least one
            // member tag was detected

            for (unsigned int j=0; j<tag_bundle_descriptions_.size(); j++)
            {
                // Get bundle name
                std::string bundleName = tag_bundle_descriptions_[j].name();

                std::map<std::string,
                        std::vector<cv::Point3d> >::iterator it =
                    bundleObjectPoints.find(bundleName);
                if (it != bundleObjectPoints.end())
                {
                    // Some member tags of this bundle were detected, get the bundle's
                    // position!
                    TagBundleDescription& bundle = tag_bundle_descriptions_[j];

                    Eigen::Isometry3d transform =
                        getRelativeTransform(bundleObjectPoints[bundleName],
                                            bundleImagePoints[bundleName], fx, fy, cx, cy);
                    geometry_msgs::msg::PoseWithCovarianceStamped bundle_pose =
                        makeTagPose(transform, image->header);

                    // Add the detection to the back of the tag detection array
                    apriltag_ros2::msg::AprilTagDetection tag_detection;
                    tag_detection.pose = bundle_pose;
                    tag_detection.id = bundle.bundleIds();
                    tag_detection.size = bundle.bundleSizes();
                    tag_detection_array.detections.push_back(tag_detection);
                    detection_names.push_back(bundle.name());
                }
            }

            // If set, publish the transform /tf topic
            // if (publish_tf_) {
            //     for (unsigned int i=0; i<tag_detection_array.detections.size(); i++) {
            //         geometry_msgs::msg::PoseStamped pose;
            //         pose.pose = tag_detection_array.detections[i].pose.pose.pose;
            //         pose.header = tag_detection_array.detections[i].pose.header;

            //         geometry_msgs::msg::TransformStamped tag_transform;

            //         tf2::Stamped<tf2::Transform> tag_transform;

            //         tf2:(pose, tag_transform);
            //         tf_pub_->sendTransform(tag_transform);
            //     }
            // }

            return tag_detection_array;
        }

}

