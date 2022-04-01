#include <object_detection_ros/visionpipeline.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <object_detection_ros/arrbbox.h>
#include <object_detection_ros/bbox.h>
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
    ros::NodeHandle n;

    std::string marker_topic;
    std::string bounding_box_topic;
    std::string depth_image_topic;
    std::string maker_frame_id;

    double get_depth_at_pixel(rs2::depth_frame depth_frame, double pixel_x, double pixel_y);
    void convert_depth_pixel_to_metric_coordinate(double &X, double &Y, double &Z, double depth, double pixel_x, double pixel_y, rs2_intrinsics camera_intrinsics);
    void populate_marker_msg(visualization_msgs::MarkerArray &marker_array, std::string frame_id, std::vector<geometry_msgs::Point> objects_array, std::vector<std::string> &names);


    public:
    
    DetectorNode();
    ~DetectorNode();
    int run();


};


DetectorNode::DetectorNode(){

    marker_topic        = "object_markers";
    bounding_box_topic  = "detection_bbox";
    depth_image_topic   = "depth_image";
    maker_frame_id      = "camera_link";

    marker_pub          = n.advertise<visualization_msgs::MarkerArray>(marker_topic, 1);
    detection_publish   = n.advertise<object_detection_ros::arrbbox>(depth_image_topic, 1);


}

DetectorNode::~DetectorNode(){


}


int DetectorNode::run(){

    image_transport::ImageTransport it_(n);
    image_transport::Publisher image_pub_ = it_.advertise(depth_image_topic, 1);
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
      
    rs2::pipeline pipe;     
	rs2::config cfg;        
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	rs2::pipeline_profile selection = pipe.start(cfg);
    auto const intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
	
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);
	float       alpha = 0.5f; 
	rs2::colorizer c;
	direction   dir = direction::to_color;  // Alignment direction

    VisionPipeline pipeline;

	bool stop = false;

	while (ros::ok())
	{

		rs2::frameset frameset = pipe.wait_for_frames();
        
		if (dir == direction::to_depth)
        {
            // Align all frames to depth viewport
            frameset = align_to_depth.process(frameset);
        }
        else
        {
            // Align all frames to color viewport
            frameset = align_to_color.process(frameset);
        }

		auto depth_frame = frameset.get_depth_frame();
        auto color_frame = frameset.get_color_frame();
        auto colorized_depth = c.colorize(depth_frame);

		cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_alligned = cv::Mat(cv::Size(640, 480), CV_16SC1, const_cast<void*>(depth_frame.get_data()));
        
        std::vector<cv::Point> objects;
        std::vector<std::string> names;
        std::vector<geometry_msgs::Point> objects_array;
        std::vector<std::array<float,4>> detected_boxes;
        std::vector<float> detected_scores;

        pipeline.run(color).getObjects(objects).getNames(names).getScore(detected_scores).getBoundingBoxes(detected_boxes);
        object_detection_ros::arrbbox arr_bbox_msg;
        
        int i = 0;
        for(cv::Point o : objects){

            geometry_msgs::Point p;
            object_detection_ros::bbox bbox_msg;

            double x_cam, y_cam, z_cam;
            
            double depth = get_depth_at_pixel(depth_frame, o.x, o.y);
            convert_depth_pixel_to_metric_coordinate(x_cam,y_cam,z_cam, depth, o.x,o.y, intrinsics);
            std::cout << i << ":" << x_cam << "," << y_cam<< "," << z_cam <<"->" << names.at(i) << std::endl;
            p.x = z_cam;
            p.y = -1*x_cam;
            p.z = -1*y_cam;
            std::cout << "------------------------------------------------" << std::endl;
            objects_array.push_back(p);
            
            bbox_msg.class_name = names.at(i);
            bbox_msg.score = detected_scores.at(i);

            try{
                if (detected_boxes.at(i).size() == 4){
                    for (int j = 0 ; j < detected_boxes.at(i).size(); j++){
                        bbox_msg.bbox.push_back(detected_boxes.at(i).at(j));
                    }

                    arr_bbox_msg.detected.push_back(bbox_msg);
                }
                
            }
            catch(...){
                std::cout << "Unknown exemption !" << std::endl;
                //bbox_msg.bbox.clear();
            }            
            i++;
        }
        if (objects_array.size() > 0){
            visualization_msgs::MarkerArray marker_array;
            populate_marker_msg(marker_array,maker_frame_id, objects_array, names);
            marker_pub.publish(marker_array);
            detection_publish.publish(arr_bbox_msg);
            
        }
        arr_bbox_msg.detected.clear();

        if ((image_pub_.getNumSubscribers() > 0)){

            cv_ptr->encoding = "mono16";
            cv_ptr->header.stamp = ros::Time::now();
            cv_ptr->header.frame_id = "/depth_alligned";
            cv_ptr->image = depth_alligned;
            image_pub_.publish(cv_ptr->toImageMsg());
        }
        
        imshow("Color Image", color);
        if (cv::waitKey(10) >= 0)
            stop = true;
        
        while (marker_pub.getNumSubscribers() < 1){
            if (!ros::ok()){
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
        }
    

	}
	return 0;
}


double DetectorNode::get_depth_at_pixel(rs2::depth_frame depth_frame, double pixel_x, double pixel_y){

	return depth_frame.get_distance(round(pixel_x), round(pixel_y));
 }


void DetectorNode::convert_depth_pixel_to_metric_coordinate(double &X, double &Y, double &Z, double depth, double pixel_x, double pixel_y, rs2_intrinsics camera_intrinsics){

	X = (pixel_x - camera_intrinsics.ppx)/camera_intrinsics.fx *depth;
	Y = (pixel_y - camera_intrinsics.ppy)/camera_intrinsics.fy *depth;
    Z = depth;
}


void DetectorNode::populate_marker_msg(visualization_msgs::MarkerArray &marker_array, std::string frame_id, std::vector<geometry_msgs::Point> objects_array, std::vector<std::string> &names){

    geometry_msgs::Pose pose_in_map_frame;
    
    if  (frame_id == "map"){

        try{

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            pose_in_map_frame = tfBuffer.lookupTransform("camera_link", "map", ros::Time(0));

        } 

        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

    }
    
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker_array.markers.clear();
    int counter = 0;
    for(auto p : objects_array){
        visualization_msgs::Marker marker;
        visualization_msgs::Marker marker_text;

        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();

        marker_text.header = marker.header;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one

        marker.id = counter;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.id = 100+counter;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
        marker_text.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = p.x;
        marker.pose.position.y = p.y;
        marker.pose.position.z = p.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker_text.pose.position.x = p.x;
        marker_text.pose.position.y = p.y;
        marker_text.pose.position.z = p.z + 0.5;
        marker_text.pose.orientation.x = 0.0;
        marker_text.pose.orientation.y = 0.0;
        marker_text.pose.orientation.z = 0.0;
        marker_text.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        marker_text.scale.x = 0.5;
        marker_text.scale.y = 0.5;
        marker_text.scale.z = 0.5;


        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_text.color.r = 1.0f;
        marker_text.color.g = 1.0f;
        marker_text.color.b = 0.0f;
        marker_text.color.a = 1.0;
        
        marker_text.text = names.at(counter);
           
        marker_text.lifetime = ros::Duration(1);
        marker.lifetime = ros::Duration(1);
        marker_array.markers.push_back(marker);
        marker_array.markers.push_back(marker_text);
        counter++;
        
    }
    return; 
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detection");
    DetectorNode node;
    node.run();

}