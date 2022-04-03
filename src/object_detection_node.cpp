#include <object_detection_ros/detectornode.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detection");
    DetectorNode node;
    node.run();

}