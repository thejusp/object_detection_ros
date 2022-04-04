#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <object_detection_ros/Object.h>
#include <object_detection_ros/ObjectArray.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


class ObjectHandler{

    private:

    ros::NodeHandle node;
    ros::Subscriber sub;
    ros::Publisher marker_pub;

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped camera_pose;
    geometry_msgs::PoseStamped map_pose;

    object_detection_ros::ObjectArray objects_in_cam;
    object_detection_ros::ObjectArray objects_received;
    bool is_objects_received{false};
    bool is_transform_received{false};

    tf2_ros::Buffer tfBuffer;

    void objectCB(const object_detection_ros::ObjectArray& msg);
    void publishMarker();


    public:

    ObjectHandler();
    void run();


};


ObjectHandler::ObjectHandler(){

    sub = node.subscribe("objects", 1, &ObjectHandler::objectCB, this);
    marker_pub = node.advertise<visualization_msgs::MarkerArray>("markers", 1);

};




void ObjectHandler::objectCB(const object_detection_ros::ObjectArray& msg){
    
    objects_in_cam = msg;
    is_objects_received = true;
    if (is_transform_received){
        objects_received.object_array.clear();

        for(auto p : objects_in_cam.object_array){
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::PoseStamped map_pose, poseIn;
            object_detection_ros::Object obj;
            poseIn.header.frame_id = "camera_link";
            poseIn.pose = p.pose;

            objects_received.header.frame_id = "map";

            try{
                map_pose = tfBuffer.transform(poseIn, "map", ros::Duration(0.0));
                obj.pose = map_pose.pose;
                obj.name = p.name;
                objects_received.object_array.push_back(obj);
                //std::cout << "Pose updated to map frame" << std::endl;
                

            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("objectCB %s",ex.what());
    
            }
            objects_received.object_array.push_back(obj);
        }
        publishMarker();
    }
}



void ObjectHandler::publishMarker(){

    uint16_t counter = 0;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_text;

    if(objects_received.object_array.size() == 0){
        std::cout << "Empty Marker !!" << std::endl;
        return;
    }
    
    for(auto p : objects_received.object_array){

        // std::cout << "Adding objects: " << p.name << std::endl; 

        marker.id           = counter;
        marker_text.id      = 100+counter;

        marker.type         = visualization_msgs::Marker::CYLINDER;
        marker_text.type    = visualization_msgs::Marker::TEXT_VIEW_FACING;
        
        marker.action = visualization_msgs::Marker::ADD;
        marker_text.action = visualization_msgs::Marker::ADD;

        //Marker for object
        marker.header.frame_id          = objects_received.header.frame_id;
        marker.header.stamp             = objects_received.header.stamp;
        marker.pose                     = p.pose;
        //Marker for text

        marker_text.header.frame_id     = objects_received.header.frame_id;
        marker_text.header.stamp        = objects_received.header.stamp;
        marker_text.pose.orientation    = p.pose.orientation;
        marker_text.pose.position       = p.pose.position;
        marker_text.pose.position.z     = p.pose.position.z + 0.5; // offset the text display over z

        // Add colors

        marker.color.r      = 0.0f;
        marker.color.g      = 1.0f;
        marker.color.b      = 1.0f;
        marker.color.a      = 1.0;

        marker_text.color.r = 1.0f;
        marker_text.color.g = 1.0f;
        marker_text.color.b = 0.0f;
        marker_text.color.a = 1.0;

        // add scale 

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        marker_text.scale.x = 0.5;
        marker_text.scale.y = 0.5;
        marker_text.scale.z = 0.5;

        marker_text.text        = p.name.data.c_str();
        
        marker_text.lifetime    = ros::Duration();
        marker.lifetime         = ros::Duration();

        marker_array.markers.push_back(marker);
        marker_array.markers.push_back(marker_text);
        counter++;
    
    }
    marker_pub.publish(marker_array);
    return; 
}

void ObjectHandler::run(){
    
    tf2_ros::TransformListener tfListener(tfBuffer);



    ros::Rate rate(5.0);
    while (node.ok()){
        try{
            transformStamped = tfBuffer.lookupTransform("camera_link", "map", ros::Time(0));
            is_transform_received = true;
        }
        catch(tf2::TransformException &ex){
            
            ROS_WARN("Run %s",ex.what());
            ros::Rate(1).sleep();
            is_transform_received = false;
            continue;
        }   
        rate.sleep();
        ros::spinOnce();
    }
}



int main(int argc, char** argv){

    ros::init(argc, argv, "object_handler");
    ObjectHandler node;
    node.run();

}
