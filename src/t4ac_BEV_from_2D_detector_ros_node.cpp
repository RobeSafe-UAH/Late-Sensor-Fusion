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
#include <numeric>
#include <exception>

// ROS includes

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <sensor_msgs/Image.h>
#include <derived_object_msgs/ObjectArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>

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
                    BEV_image_detections_marker_topic, depth_map_topic, 
                    image_detections_topic, laser_frame, map_frame;
        
        double fov;
        double markers_lifetime;
        int queue_size_pubs, queue_size_subs;

        ros::NodeHandle nh;

        ros::Publisher pub_detected_obstacles;
        ros::Publisher pub_detected_road_signals;
        ros::Publisher pub_detected_bev_image_detections_marker_array;
        ros::Publisher pub_groundtruth_objects_marker_array;

        ros::Subscriber sub_carla_objects;

        message_filters::Subscriber<sensor_msgs::Image> depth_map_sub;
        message_filters::Subscriber<t4ac_msgs::Bounding_Box_2D_list> image_detections_sub;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, t4ac_msgs::Bounding_Box_2D_list> MySyncPolicy;

        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;   

        tf::TransformListener *listener;
        tf::StampedTransform tf_lidar2map;
    public:
        // Constructor

        Node(ros::NodeHandle nh)
        {
            // Specify private parameters

            this->nh = nh;
            
            queue_size_pubs = 10;
            queue_size_subs = 20; // 40

            markers_lifetime = 0.0;

            // Camera parameters

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/horizontal_fov", fov);
            
            // ROS publishers

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/pub_BEV_image_obstacles", BEV_image_obstacles_topic);
            pub_detected_obstacles = nh.advertise<t4ac_msgs::BEV_detections_list>(BEV_image_obstacles_topic, queue_size_pubs, true);

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/pub_BEV_road_signals_obstacles", BEV_image_road_signals_topic);
            pub_detected_road_signals = nh.advertise<t4ac_msgs::BEV_detections_list>(BEV_image_road_signals_topic, queue_size_pubs, true);

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/pub_BEV_image_detections_markers", BEV_image_detections_marker_topic);
            pub_detected_bev_image_detections_marker_array = nh.advertise<visualization_msgs::MarkerArray>(BEV_image_detections_marker_topic, queue_size_pubs, true);

            pub_groundtruth_objects_marker_array = nh.advertise<visualization_msgs::MarkerArray>("/carla/ego_vehicle/objects_marker", queue_size_pubs, true);

            // ROS subscribers

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/sub_image_depth_map", depth_map_topic);
            depth_map_sub.subscribe(nh, depth_map_topic, queue_size_subs);

            nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_BEV_from_2D_detector_ros_node/sub_image_detections", image_detections_topic);
            image_detections_sub.subscribe(nh, image_detections_topic, queue_size_subs);

            sub_carla_objects = nh.subscribe("/carla/ego_vehicle/objects", queue_size_subs, &Node::carla_objects_callback, this);

            // Get frames

            nh.getParam("/t4ac/frames/laser", laser_frame);
            nh.getParam("/t4ac/frames/map", map_frame);

            // Main callback

            sync_.reset(new Sync(MySyncPolicy(200), depth_map_sub, image_detections_sub));
            sync_->registerCallback(boost::bind(&Node::bev_from_2d_object_detector_callback, this, _1, _2)); 

            // Transform listener

	        listener = new tf::TransformListener(ros::Duration(5.0));
        }

        template<typename T>
        std::vector<T> sub_vector(std::vector<T> const &v, int m, int n) 
        {
            auto first = v.begin() + m;
            auto last = v.begin() + n + 1;
            std::vector<T> new_vector(first, last);
            return new_vector;
        }

        double compute_median(std::vector<double> bounding_box_vector)
        {
            std::nth_element(bounding_box_vector.begin(), bounding_box_vector.begin() + bounding_box_vector.size()/2, bounding_box_vector.end());
            double median = bounding_box_vector[bounding_box_vector.size()/2];

            std::cout << "The median (red) is: " << median << std::endl;

            return median;
        }

        double compute_mean(std::vector<double> bounding_box_vector)
        {
            std::sort(bounding_box_vector.begin(), bounding_box_vector.end());
            int vector_len = bounding_box_vector.size();
   
            float percentile = 0.25;
            float tolerance = percentile/4;

            int first, last;
            first = int(vector_len*percentile*(1-tolerance));
            last = int(vector_len*percentile*(1+tolerance));

            std::vector<double> median_vector = sub_vector(bounding_box_vector, first, last+1);

            double sum = std::accumulate(median_vector.begin(), median_vector.end(), 0.0);
            double mean = sum / median_vector.size();

            std::cout << "The mean (green) is: " << mean << std::endl;

            return mean;
        }

        double compute_mode(std::vector<double> mode_vector)
        {
            // std::sort(bounding_box_vector.begin(), bounding_box_vector.end());
            // int vector_len = bounding_box_vector.size();
            // float tolerance = 0.2;
            // int first, last;
            // first = int(vector_len*(0.5 - tolerance));
            // last = int(vector_len*(0.5 + tolerance));

            // std::vector<double> mode_vector = sub_vector(bounding_box_vector, first, last+1);

            for (size_t i=0; i<mode_vector.size(); i++)
            {
                mode_vector[i] = int(round(mode_vector[i]*100.0));
            } 

            int max = mode_vector.back();
            int min = mode_vector.front();
            int prev = max;
            int mode;
            int maxcount = 0;
            int currcount = 0;
            for (auto n : mode_vector) {
                if (n == prev) {
                    ++currcount;
                    if (currcount > maxcount) {
                        maxcount = currcount;
                        mode = n;
                    }
                } else {
                    currcount = 1;
                }
                prev = n;
            }
            float mode_aux = float(mode);
            mode_aux /= 100.0f;

            std::cout << "The mode (dark blue) is: " << mode_aux << std::endl;

            return mode_aux;
        }

        double compute_centroid(std::vector<double> bounding_box_vector)
        {
            double z_centroid = bounding_box_vector[bounding_box_vector.size()/2];

            std::cout << "The centroid (yellow) is: " << z_centroid << std::endl;

            return z_centroid;
        }

        void bev_xy(float z, double f, Point bounding_box_centroid, 
                    Point image_center, t4ac_msgs::BEV_detection& bev_image_detection)
        {
            // TODO: Calibrate LiDAR and Camera
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

        geometry_msgs::Point32 Global_To_Local_Coordinates(geometry_msgs::Point32 point_global)
        {
            tf::Vector3 aux, aux2;
            geometry_msgs::Point32 pt_local;

            aux.setX(point_global.x);
            aux.setY(point_global.y);
            aux.setZ(point_global.z);

            aux2 = tf_lidar2map * aux;

            pt_local.x = aux2.getX();
            pt_local.y = aux2.getY();
            pt_local.z = aux2.getZ();

            return(pt_local);
        }

        void carla_objects_callback(const derived_object_msgs::ObjectArray::ConstPtr& carla_objects_msg)
        {
            visualization_msgs::MarkerArray gt_marker_array;

            for (size_t i=0; i<carla_objects_msg->objects.size(); i++)
            {
                geometry_msgs::Point32 pt_global, pt_local;
                pt_global.x = carla_objects_msg->objects[i].pose.position.x;
                pt_global.y = carla_objects_msg->objects[i].pose.position.y;
                pt_global.z = carla_objects_msg->objects[i].pose.position.z;

                pt_local = Global_To_Local_Coordinates(pt_global); 

                visualization_msgs::Marker gt_marker;

                gt_marker.header.frame_id = laser_frame;
                gt_marker.ns = "bev_image_detections";
                gt_marker.id = i;
                gt_marker.action = visualization_msgs::Marker::ADD;
                gt_marker.type = visualization_msgs::Marker::SPHERE;
                gt_marker.lifetime = ros::Duration(markers_lifetime);
                gt_marker.pose.position.x = pt_local.x;
                gt_marker.pose.position.y = pt_local.y;
                gt_marker.pose.position.z = -1.5;
                gt_marker.scale.x = 0.5;
                gt_marker.scale.y = 0.5;
                gt_marker.scale.z = 0.5;
                gt_marker.color.r = 1.0;
                gt_marker.color.g = 1.0;
                gt_marker.color.b = 1.0;
                gt_marker.color.a = 1.0;

                gt_marker_array.markers.push_back(gt_marker);
            }

            pub_groundtruth_objects_marker_array.publish(gt_marker_array);
        }

        void bev_from_2d_object_detector_callback(const sensor_msgs::Image::ConstPtr& depth_msg, 
                                                  const t4ac_msgs::Bounding_Box_2D_list::ConstPtr& image_detections_msg)
        {
            try
            {
                listener->lookupTransform(laser_frame, map_frame, depth_msg->header.stamp, tf_lidar2map);
            }
            catch(tf::TransformException& e)
            {
                std::cout << e.what();
                return; 
            }
            std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
            t4ac_msgs::BEV_detections_list bev_image_obstacles, bev_image_road_obstacles;
            visualization_msgs::MarkerArray bev_image_detections_marker_array;

            std::string traffic_string = "traffic"; // Traffic signal and traffic light
            std::string stop_string = "stop"; // Stop signal (vertical and road)
            cv::Mat image_depth;

            Point image_center(depth_msg->width/2, depth_msg->height/2);
            float f = depth_msg->width / (2 * tan(fov * M_PI / 360));    
            int id_marker = 0;

            bev_image_obstacles.header.stamp = bev_image_road_obstacles.header.stamp = depth_msg->header.stamp;

            bev_image_obstacles.front = 30;
            bev_image_obstacles.back = -10;
            bev_image_obstacles.left = -15;
            bev_image_obstacles.right = 15;

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

                    int bb_width = x2-x1;
                    int bb_height = y2-y1;

                    float ratio = 0.2;

                    int kw = round(bb_width*ratio); // kernel_width
                    int kh = round(bb_height*ratio); // kernel height
                    int xc = (x1+x2)/2;
                    int yc = (y1+y2)/2;
                    Point bounding_box_centroid(xc, yc);

                    std::cout << "................." << std::endl;

                    cv::Mat aux;

                    int min_side = 30;

                    if (x2-x1 >= min_side && y2-y1 >= min_side)
                    {
                        cv::Mat aux2 = image_depth(cv::Range(yc-kh/2,yc+kh/2),cv::Range(xc-kw/2,xc+kw/2));
                        std::cout << "KERNEL" << std::endl;
                        aux2.copyTo(aux);
                    }
                    else
                    {
                        cv::Mat aux2 = image_depth(cv::Range(y1,y2),cv::Range(x1,x2));
                        std::cout << "WHOLE" << std::endl;
                        aux2.copyTo(aux);
                    }

                    bb_width = aux.cols;
                    bb_height = aux.rows;
                    aux = aux.reshape(0,1); // Single row, concatenating rows of the original matrix
            
                    std::vector<double> bounding_box_vector;
                    aux.copyTo(bounding_box_vector);

                    bounding_box_vector.erase(std::remove_if(std::begin(bounding_box_vector),
                                                             std::end(bounding_box_vector),
                                                             [](const auto& value) { return (std::isnan(value) || std::isinf(value)); }),
                                              std::end(bounding_box_vector));

                    for (size_t i=0; i<bounding_box_vector.size(); i++)
                    {
                        std::cout << bounding_box_vector[i] << " ";
                    }

                    // BEV image detection (a.k.a. LiDAR frame, z-filtered)

                    t4ac_msgs::BEV_detection bev_image_detection;

                    std::string bb_type = image_detections_msg->bounding_box_2D_list[i].type;
                    bev_image_detection.type = bb_type;
                    bev_image_detection.score = image_detections_msg->bounding_box_2D_list[i].score;

                    // REMOVE
                    float side = 1.0;
					bev_image_detection.x_corners = {-side,side,-side,side};
					bev_image_detection.y_corners = {-side,-side,side,side};
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

                         // Visual Marker

                    float height = -1.5;
                    int num_marker = 0;
                    float step = 0.1;

                    double z = compute_median(bounding_box_vector);
                    bev_xy(z, f, bounding_box_centroid, image_center, bev_image_detection);

                    visualization_msgs::Marker bev_image_detection_marker;

                    bev_image_detection_marker.header.frame_id = laser_frame;
                    bev_image_detection_marker.ns = "bev_image_detections";
                    bev_image_detection_marker.id = id_marker;
                    id_marker++;
                    bev_image_detection_marker.action = visualization_msgs::Marker::ADD;
                    bev_image_detection_marker.type = visualization_msgs::Marker::CUBE;
                    bev_image_detection_marker.lifetime = ros::Duration(markers_lifetime);
                    bev_image_detection_marker.pose.position.x = bev_image_detection.x;
                    bev_image_detection_marker.pose.position.y = bev_image_detection.y;
                    bev_image_detection_marker.pose.position.z = height+num_marker*step;
                    bev_image_detection_marker.scale.x = 0.5;
                    bev_image_detection_marker.scale.y = 0.5;
                    bev_image_detection_marker.scale.z = 0.5;
                    bev_image_detection_marker.color.r = 1.0;
                    bev_image_detection_marker.color.g = 0.0;
                    bev_image_detection_marker.color.b = 0.0;
                    bev_image_detection_marker.color.a = 1.0;

                    // bev_image_detections_marker_array.markers.push_back(bev_image_detection_marker);

                    num_marker++;

                    //

                    z = compute_mean(bounding_box_vector);
                    bev_xy(z, f, bounding_box_centroid, image_center, bev_image_detection);

                    visualization_msgs::Marker mean;

                    mean.header.frame_id = laser_frame;
                    mean.ns = "bev_image_detections";
                    mean.id = id_marker;
                    id_marker++;
                    mean.action = visualization_msgs::Marker::ADD;
                    mean.type = visualization_msgs::Marker::CUBE;
                    mean.lifetime = ros::Duration(markers_lifetime);
                    mean.pose.position.x = bev_image_detection.x;
                    mean.pose.position.y = bev_image_detection.y;
                    mean.pose.position.z = height+num_marker*step;
                    mean.scale.x = 0.5;
                    mean.scale.y = 0.5;
                    mean.scale.z = 0.5;
                    mean.color.r = 0.0;
                    mean.color.g = 1.0;
                    mean.color.b = 0.0;
                    mean.color.a = 1.0;

                    bev_image_detections_marker_array.markers.push_back(mean);

                    num_marker++;

                    //

                    z = compute_mode(bounding_box_vector);
                    bev_xy(z, f, bounding_box_centroid, image_center, bev_image_detection);

                    visualization_msgs::Marker mode;

                    mode.header.frame_id = laser_frame;
                    mode.ns = "bev_image_detections";
                    mode.id = id_marker;
                    id_marker++;
                    mode.action = visualization_msgs::Marker::ADD;
                    mode.type = visualization_msgs::Marker::CUBE;
                    mode.lifetime = ros::Duration(markers_lifetime);
                    mode.pose.position.x = bev_image_detection.x;
                    mode.pose.position.y = bev_image_detection.y;
                    mode.pose.position.z = height+num_marker*step;
                    mode.scale.x = 0.5;
                    mode.scale.y = 0.5;
                    mode.scale.z = 0.5;
                    mode.color.r = 0.0;
                    mode.color.g = 0.0;
                    mode.color.b = 1.0;
                    mode.color.a = 1.0;

                    // bev_image_detections_marker_array.markers.push_back(mode);

                    num_marker++;

                    //

                    z = compute_centroid(bounding_box_vector);
                    bev_xy(z, f, bounding_box_centroid, image_center, bev_image_detection);

                    visualization_msgs::Marker centroid;

                    centroid.header.frame_id = laser_frame;
                    centroid.ns = "bev_image_detections";
                    centroid.id = id_marker;
                    id_marker++;
                    centroid.action = visualization_msgs::Marker::ADD;
                    centroid.type = visualization_msgs::Marker::CUBE;
                    centroid.lifetime = ros::Duration(markers_lifetime);
                    centroid.pose.position.x = bev_image_detection.x;
                    centroid.pose.position.y = bev_image_detection.y;
                    centroid.pose.position.z = height+num_marker*step;
                    centroid.scale.x = 0.5;
                    centroid.scale.y = 0.5;
                    centroid.scale.z = 0.5;
                    centroid.color.r = 1.0;
                    centroid.color.g = 1.0;
                    centroid.color.b = 0.0;
                    centroid.color.a = 1.0;

                    // bev_image_detections_marker_array.markers.push_back(centroid);

                    // 

                    num_marker++;

                    std::cout << "Height: " << bb_height << " Width: " << bb_width << " Size: " << bounding_box_vector.size() << std::endl;
                    std::vector<double> lowest_vector = sub_vector(bounding_box_vector, (bb_height-1)*bb_width, bounding_box_vector.size());
                    std::cout << "After sub" << std::endl;
                    // for (size_t i=0; i<lowest_vector.size(); i++)
                    // {
                    //     std::cout << lowest_vector[i] << " ";
                    // }
                    // std::vector<double> lowest_vector = bounding_box_vector;

                    z = compute_median(lowest_vector);
                    std::cout << "The median of lowest vector (purple) is: " << z << std::endl;
                    bev_xy(z, f, bounding_box_centroid, image_center, bev_image_detection);

                    visualization_msgs::Marker lowest;

                    lowest.header.frame_id = laser_frame;
                    lowest.ns = "bev_image_detections";
                    lowest.id = id_marker;
                    id_marker++;
                    lowest.action = visualization_msgs::Marker::ADD;
                    lowest.type = visualization_msgs::Marker::CUBE;
                    lowest.lifetime = ros::Duration(markers_lifetime);
                    lowest.pose.position.x = bev_image_detection.x;
                    lowest.pose.position.y = bev_image_detection.y;
                    lowest.pose.position.z = height+num_marker*step;
                    lowest.scale.x = 0.5;
                    lowest.scale.y = 0.5;
                    lowest.scale.z = 0.5;
                    lowest.color.r = 1.0;
                    lowest.color.g = 0.0;
                    lowest.color.b = 1.0;
                    lowest.color.a = 1.0;

                    // bev_image_detections_marker_array.markers.push_back(lowest);

                    } 
                }    
            }

            // pub_detected_road_signals.publish(bev_image_road_obstacles);
            // pub_detected_obstacles.publish(bev_image_obstacles);  
            pub_detected_bev_image_detections_marker_array.publish(bev_image_detections_marker_array);
        }
};

int main(int argc, char **argv)
{
    // Init ROS node

    ros::init(argc, argv, "t4ac_BEV_from_2D_detector_ros_node"); 
    ros::NodeHandle nh;

    Node synchronizer(nh);

    // ROS spin

    ros::spin();
}