#include "apriltag_ros2/common_functions.h"

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

}

