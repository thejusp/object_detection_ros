#include <object_detection_ros/detectornode.h>


DetectorNode::DetectorNode(){

    marker_topic        = "object_markers";
    bounding_box_topic  = "detection_bbox";
    depth_image_topic   = "depth_image";
    maker_frame_id      = "camera_link";

    show_preview        = true;

    marker_life_time    = 5.0;
    marker_pub          = n.advertise<visualization_msgs::MarkerArray>(marker_topic, 1);
    detection_publish   = n.advertise<object_detection_ros::arrbbox>(depth_image_topic, 1);


}

DetectorNode::~DetectorNode(){


}


int DetectorNode::run(){

    image_transport::ImageTransport it_(n);
    image_transport::Publisher      image_pub_ = it_.advertise(depth_image_topic, 1);
    cv_bridge::CvImagePtr           cv_ptr(new cv_bridge::CvImage);
      
    rs2::pipeline pipe;     
	rs2::config   cfg;        
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	rs2::pipeline_profile selection = pipe.start(cfg);
    auto const intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
	
	rs2::align     align_to_depth(RS2_STREAM_DEPTH);
    rs2::align     align_to_color(RS2_STREAM_COLOR);
	float          alpha = 0.5f; 
	rs2::colorizer c;
	direction      dir = direction::to_color;  // Alignment direction

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

		auto depth_frame     = frameset.get_depth_frame();
        auto color_frame     = frameset.get_color_frame();
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
            //std::cout << i << ":" << x_cam << "," << y_cam<< "," << z_cam <<"->" << names.at(i) << std::endl;
            p.x = z_cam;
            p.y = -1*x_cam;
            p.z = -1*y_cam;
            //std::cout << "------------------------------------------------" << std::endl;
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

	}
	return 0;
}

bool DetectorNode::trasnformPoint(geometry_msgs::PoseStamped &poseIn, geometry_msgs::PoseStamped &poseOut, std::string to_frame){

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    

    try{
        transformStamped = tfBuffer.lookupTransform("camera_link", "map", ros::Time(0));
        poseOut = tfBuffer.transform(poseIn, to_frame, ros::Duration(0.0));
        return true;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
        return false;
    }
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

    uint16_t counter;
    marker_array.markers.clear();
    for(auto p : objects_array){

        geometry_msgs::PoseStamped poseIn; 

        poseIn.header.frame_id = "camera_link";
        poseIn.header.stamp = ros::Time::now();
        poseIn.pose.position.x = p.x;
        poseIn.pose.position.y = p.y;
        poseIn.pose.position.z = p.z;
        poseIn.pose.orientation.x = 0.0;
        poseIn.pose.orientation.y = 0.0;
        poseIn.pose.orientation.z = 0.0;
        poseIn.pose.orientation.w = 1.0;

        visualization_msgs::Marker marker;
        visualization_msgs::Marker marker_text;

        if(frame_id != "camera_link"){

            geometry_msgs::PoseStamped poseOut;
            if(trasnformPoint(poseIn, poseOut, frame_id)){

                marker.id           = counter;
                marker_text.id      = 100+counter;

                marker.type         = visualization_msgs::Marker::CYLINDER;
                marker_text.type    = visualization_msgs::Marker::TEXT_VIEW_FACING;
                
                marker.action = visualization_msgs::Marker::ADD;
                marker_text.action = visualization_msgs::Marker::ADD;

                //Marker for object
                marker.header.frame_id          = poseOut.header.frame_id;
                marker.header.stamp             = poseOut.header.stamp;
                marker.pose                     = poseOut.pose;
                //Marker for text

                marker_text.header.frame_id     = poseOut.header.frame_id;
                marker_text.header.stamp        = poseOut.header.stamp;
                marker_text.pose.orientation    = poseOut.pose.orientation;
                marker_text.pose.position       = poseOut.pose.position;
                marker_text.pose.position.z     = poseOut.pose.position.z + 0.5; // offset the text display over z

                // Add colors

                marker.color.r      = 0.0f;
                marker.color.g      = 1.0f;
                marker.color.b      = 0.0f;
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

                marker_text.text        = names.at(counter);
                marker_text.lifetime    = ros::Duration(1);
                marker.lifetime         = ros::Duration(1);

                marker_array.markers.push_back(marker);
                marker_array.markers.push_back(marker_text);
                counter++;

            }
        }
        else {

            marker.header.frame_id          = poseIn.header.frame_id;
            marker.header.stamp             = poseIn.header.stamp;
            marker.pose                     = poseIn.pose;

            marker.id           = counter;
            marker_text.id      = 100+counter;

            marker.type         = visualization_msgs::Marker::CYLINDER;
            marker_text.type    = visualization_msgs::Marker::TEXT_VIEW_FACING;
            
            marker.action = visualization_msgs::Marker::ADD;
            marker_text.action = visualization_msgs::Marker::ADD;
                        
            //Marker for text

            marker_text.header.frame_id     = poseIn.header.frame_id;
            marker_text.header.stamp        = poseIn.header.stamp;
            marker_text.pose.orientation    = poseIn.pose.orientation;
            marker_text.pose.position       = poseIn.pose.position;
            marker_text.pose.position.z     = poseIn.pose.position.z + 0.5; // offset the text display over z

            // Add colors

            marker.color.r                  = 0.0f;
            marker.color.g                  = 1.0f;
            marker.color.b                  = 0.0f;
            marker.color.a                  = 1.0;

            marker_text.color.r             = 1.0f;
            marker_text.color.g             = 1.0f;
            marker_text.color.b             = 0.0f;
            marker_text.color.a             = 1.0;

            // add scale 

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            marker_text.scale.x = 0.5;
            marker_text.scale.y = 0.5;
            marker_text.scale.z = 0.5;

            marker_text.text                = names.at(counter);
        
            marker_text.lifetime            = ros::Duration(marker_life_time);
            marker.lifetime                 = ros::Duration(marker_life_time);
            marker_array.markers.push_back(marker);
            marker_array.markers.push_back(marker_text);
            counter++;


        }
        
    }

    return; 
}
