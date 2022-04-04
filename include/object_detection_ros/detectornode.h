#ifndef __DETECTOR_NODE_H
#define __DETECTOR_NODE_H

#include <object_detection_ros/visionpipeline.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <object_detection_ros/arrbbox.h>
#include <object_detection_ros/bbox.h>
#include <object_detection_ros/Object.h>
#include <object_detection_ros/ObjectArray.h>

#include <ros/ros.h>
#include <array>

enum class direction
{
    to_depth,
    to_color
};

class DetectorNode{

    private:

    ros::Publisher marker_pub;
    ros::Publisher detection_publish;
    ros::Publisher object_pub;
    ros::NodeHandle n;

    std::string marker_topic;
    std::string bounding_box_topic;
    std::string depth_image_topic;
    std::string maker_frame_id;
    std::string object_topic;

    double marker_life_time;
    bool show_preview;

    double get_depth_at_pixel(rs2::depth_frame depth_frame, double pixel_x, double pixel_y);
    void convert_depth_pixel_to_metric_coordinate(double &X, double &Y, double &Z, double depth, double pixel_x, double pixel_y, rs2_intrinsics camera_intrinsics);
    void populate_marker_msg(visualization_msgs::MarkerArray &marker_array, std::string frame_id, std::vector<geometry_msgs::Point> objects_array, std::vector<std::string> &names);
    bool trasnformPoint(geometry_msgs::PoseStamped &poseIn, geometry_msgs::PoseStamped &poseOut,std::string to_frame);
    void populate_object_msg(object_detection_ros::ObjectArray &object_array, std::vector<geometry_msgs::Point> objects_array, std::vector<std::string> &names);


    public:
    
    DetectorNode();
    ~DetectorNode();
    int run();
};

#endif