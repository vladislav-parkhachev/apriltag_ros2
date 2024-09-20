#include <apriltag.h>
#include <cv_bridge/cv_bridge.h>

namespace apriltag_ros2
{
    class TagDetector
    {

    private:
    
    public:
        TagDetector();
        ~TagDetector();

        void drawDetections(cv_bridge::CvImagePtr image);
    };
}