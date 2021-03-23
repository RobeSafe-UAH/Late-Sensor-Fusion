/*
Created on Thu Aug  6 11:27:43 2020

@author: Carlos Gómez-Huélamo, Rodrigo Gutiérrez Moreno and Javier Araluce Ruiz

Code to 

Communications are based on ROS (Robot Operating Sytem)

Inputs: 
Outputs: 

Note that 

Executed via 
*/

// General purpose includes

#include <math.h>

// ROS includes

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Custom includes

#include "t4ac_msgs/Bounding_Box_2D.h"
#include "t4ac_msgs/Bounding_Box_2D_list.h"
#include "t4ac_msgs/BEV_detection.h"
#include "t4ac_msgs/BEV_detections_list.h"
#include <Point.hpp>

// OpenCV includes

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Node
{
    private:
        std::string BEV_image_obstacles_topic, BEV_image_road_signals_topic, 
                    BEV_image_detections_marker, depth_map_topic, 
                    image_detections_topic, frame_id;
        
        double fov;
        int queue_size;

        ros::NodeHandle nh;

        ros::Publisher pub_detected_obstacles;
        ros::Publisher pub_detected_road_signals;
        ros::Publisher pub_detected_bev_image_detections_marker_array;

        message_filters::Subscriber<sensor_msgs::Image> depth_map_sub;
        message_filters::Subscriber<t4ac_msgs::Bounding_Box_2D_list> image_detections_sub;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, t4ac_msgs::Bounding_Box_2D_list> MySyncPolicy;

        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;   
    public:
        // Constructor

        Node(ros::NodeHandle nh, int queue_size)
        {
            // Specify private parameters

            this->nh = nh;
            this->queue_size = queue_size;

            // Camera parameters

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/horizontal_fov", fov);
            
            // ROS publishers

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/pub_BEV_image_obstacles", BEV_image_obstacles_topic);
            pub_detected_obstacles = nh.advertise<t4ac_msgs::BEV_detections_list>(BEV_image_obstacles_topic, queue_size, true);

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/pub_BEV_road_signals_obstacles", BEV_image_road_signals_topic);
            pub_detected_road_signals = nh.advertise<t4ac_msgs::BEV_detections_list>(BEV_image_road_signals_topic, queue_size, true);

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/pub_BEV_image_detections_markers", BEV_image_detections_marker);
            pub_detected_bev_image_detections_marker_array = nh.advertise<visualization_msgs::MarkerArray>(BEV_image_detections_marker, queue_size, true);

            // ROS subscribers

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/sub_image_depth_map", depth_map_topic);
            depth_map_sub.subscribe(nh, depth_map_topic, queue_size);

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/sub_image_detections", image_detections_topic);
            image_detections_sub.subscribe(nh, image_detections_topic, queue_size);

            // Get frame_id for BEV image detections

            nh.getParam("/t4ac/frames/laser", frame_id);

            // Main callback

            sync_.reset(new Sync(MySyncPolicy(200), depth_map_sub, image_detections_sub));
            sync_->registerCallback(boost::bind(&Node::bev_from_2d_object_detector_callback, this, _1, _2)); 
        }

        double compute_median(cv::Mat bounding_box)
        {
            bounding_box = bounding_box.reshape(0,1); // Spread input bounding box to a single row
            std::vector<double> bounding_box_vector;
            bounding_box.copyTo(bounding_box_vector);
            std::nth_element(bounding_box_vector.begin(), bounding_box_vector.begin() + bounding_box_vector.size()/2, bounding_box_vector.end());

            return bounding_box_vector[bounding_box_vector.size()/2];
        }

        void depth_bbox(cv::Mat depth_bb, double f, Point bounding_box_centroid, 
                        Point image_center, t4ac_msgs::BEV_detection& bev_image_detection)
        {
            double z = compute_median(depth_bb);

            if (!isnan(z))
            {
                bev_image_detection.x = z - 0.41; // 0.41 is the x-distance between camera and lidar (lidar frame)
                bev_image_detection.y = (-(z * (bounding_box_centroid.get_x()-image_center.get_x())) / f); // 0.06 is the y-distance between the left camera and lidar (lidar frame)   
            }
            else
            {
                bev_image_detection.x = 50000;
                bev_image_detection.y = 50000;
            }
        }

        void bev_from_2d_object_detector_callback(const sensor_msgs::Image::ConstPtr& depth_msg, 
                                                  const t4ac_msgs::Bounding_Box_2D_list::ConstPtr& image_detections_msg)
        {
            // std::cout << "Publish: " << depth_msg->header.stamp << std::endl;
            t4ac_msgs::BEV_detections_list bev_image_obstacles, bev_image_road_obstacles;
            visualization_msgs::MarkerArray bev_image_detections_marker_array;

            std::string traffic_string = "traffic"; // Traffic signal and traffic light
            std::string stop_string = "stop"; // Stop signal (vertical and road)
            cv::Mat image_depth;

            Point image_center(depth_msg->width/2, depth_msg->height/2);
            double f = depth_msg->width / (2 * tan(fov * M_PI / 360));    

            // std::cout << "Image det: " << image_detections_msg->header.stamp.toNSec() << " depth: " << depth_msg->header.stamp.toNSec() << std::endl;
            bev_image_obstacles.header.stamp = bev_image_road_obstacles.header.stamp = depth_msg->header.stamp;

            // bev_image_obstacles.front = 30;
            // bev_image_obstacles.back = -10;
            // bev_image_obstacles.left = -15;
            // bev_image_obstacles.right = 15;

            for (size_t i = 0; i < image_detections_msg->bounding_box_2D_list.size(); i++)
            {
                if (image_detections_msg->bounding_box_2D_list[i].score > 0.0)
                {
                    try
                    {
                        image_depth = cv_bridge::toCvShare(depth_msg)->image;
                    }
                    catch (cv_bridge::Exception& e)
                    {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                        return;
                    }

                    int x1 = image_detections_msg->bounding_box_2D_list[i].x1;
                    int y1 = image_detections_msg->bounding_box_2D_list[i].y1;
                    int x2 = image_detections_msg->bounding_box_2D_list[i].x2;
                    int y2 = image_detections_msg->bounding_box_2D_list[i].y2;
                          
                    cv::Mat aux(image_depth, cv::Rect(x1, y1, x2-x1, y2-y1));
                    cv::Mat depth_bb;
                    aux.copyTo(depth_bb);
                    Point bounding_box_centroid((x1+x2)/2, (y1+y2)/2);

                    // BEV image detection (a.k.a. LiDAR frame, z-filtered)

                    t4ac_msgs::BEV_detection bev_image_detection;

                    std::string bb_type = image_detections_msg->bounding_box_2D_list[i].type;
                    bev_image_detection.type = bb_type;
                    bev_image_detection.score = image_detections_msg->bounding_box_2D_list[i].score;

                    depth_bbox(depth_bb, f, bounding_box_centroid, image_center, bev_image_detection);

                    // REMOVE
                    // float side = 2.0;
					// bev_image_detection.x_corners = {-side,side,-side,side};
					// bev_image_detection.y_corners = {-side,-side,side,side};
                    // float aux_aux = bev_image_detection.x;
                    // bev_image_detection.x = -bev_image_detection.y;
                    // bev_image_detection.y = -aux_aux;
                    // REMOVE

                    int pos = 0;
                    if ((bb_type.find(traffic_string,pos) != std::string::npos) || (bb_type.find(stop_string,pos) != std::string::npos))
                    {
                        bev_image_road_obstacles.bev_detections_list.push_back(bev_image_detection);
                    } 
                    else
                    {
                        bev_image_obstacles.bev_detections_list.push_back(bev_image_detection);
                    }

                    // Visual Marker

                    visualization_msgs::Marker bev_image_detection_marker;

                    bev_image_detection_marker.header.frame_id = frame_id;
                    bev_image_detection_marker.ns = "bev_image_detections";
                    bev_image_detection_marker.id = i;
                    bev_image_detection_marker.action = visualization_msgs::Marker::ADD;
                    bev_image_detection_marker.type = visualization_msgs::Marker::CUBE;
                    bev_image_detection_marker.lifetime = ros::Duration(0.30);
                    bev_image_detection_marker.pose.position.x = bev_image_detection.x;
                    bev_image_detection_marker.pose.position.y = bev_image_detection.y;
                    bev_image_detection_marker.pose.position.z = -1.5;
                    bev_image_detection_marker.scale.x = 0.5;
                    bev_image_detection_marker.scale.y = 0.5;
                    bev_image_detection_marker.scale.z = 0.5;
                    bev_image_detection_marker.color.r = 255;
                    bev_image_detection_marker.color.g = 0;
                    bev_image_detection_marker.color.b = 255;
                    bev_image_detection_marker.color.a = 0.8;

                    bev_image_detections_marker_array.markers.push_back(bev_image_detection_marker);
                }    
            }

            pub_detected_road_signals.publish(bev_image_road_obstacles);
            pub_detected_obstacles.publish(bev_image_obstacles);  
            pub_detected_bev_image_detections_marker_array.publish(bev_image_detections_marker_array);
        }
};

int main(int argc, char **argv)
{
    // Init ROS node

    ros::init(argc, argv, "t4ac_BEV_from_2D_detector_ros_node"); 
    ros::NodeHandle nh;

    int queue_size = 40;

    Node synchronizer(nh, queue_size);

    // ROS spin

    ros::spin();
}