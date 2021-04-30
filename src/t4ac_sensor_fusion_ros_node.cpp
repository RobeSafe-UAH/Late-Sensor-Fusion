/*
Created on Thu Aug  6 11:27:43 2020

@author: Carlos Gómez-Huélamo

Code to 

Communications are based on ROS (Robot Operating Sytem)

Inputs: 
Outputs: 

Note that 

Executed via 
*/

// Includes //

// General purpose includes

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include <string.h>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <vector>

// Custom includes

#include <Hungarian.hpp>
#include <Point.hpp>

// ROS includes

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Techs4AgeCar includes

#include "t4ac_msgs/BEV_detections_list.h"
#include "t4ac_msgs/BEV_trackers_list.h"

// End Includes //


// Global variables //

float ego_vel = 0;
float xmax, xmin, ymax, ymin;
float road_curvature = 0;
float max_road_curvature = 0;
float ratio;

//road_curvature = max_road_curvature = ego_vel = 0;

// End Global variables //


// ROS communications //

// Publishers

ros::Publisher pub_merged_obstacles_marker_array;
ros::Publisher pub_merged_obstacles;

ros::Publisher pub_monitorized_area;

// Subcribers

ros::Subscriber sub_road_curvature;
ros::Subscriber sub_odom;

// End ROS communications //


// Declarations of functions //

float euclidean_distance(Point , Point );
bool inside_monitorized_area(Point , float []);

// ROS Callbacks

void road_curvature_cb(const std_msgs::Float64::ConstPtr& );
void odom_cb(const nav_msgs::Odometry::ConstPtr& );
void sensor_fusion_cb(const t4ac_msgs::BEV_detections_list::ConstPtr& , 
					  const t4ac_msgs::BEV_detections_list::ConstPtr& );

// End Declarations of functions //


// Main //

int main (int argc, char ** argv)
{
	// Initialize ROS

	ros::init(argc, argv, "t4ac_sensor_fusion_ros_node");
	ros::NodeHandle nh;

	// Get max. road curvature by param

	nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_ros_node/max_road_curvature", max_road_curvature);

    // Publishers

	std::string BEV_merged_obstacles_marker_topic, BEV_merged_obstacles_topic, rectangular_monitorized_area_marker_topic;

	nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_ros_node/pub_BEV_merged_obstacles_marker", BEV_merged_obstacles_marker_topic);
	pub_merged_obstacles_marker_array = nh.advertise<visualization_msgs::MarkerArray>(BEV_merged_obstacles_marker_topic, 1, true);

	nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_ros_node/pub_BEV_merged_obstacles", BEV_merged_obstacles_topic);
	pub_merged_obstacles = nh.advertise<t4ac_msgs::BEV_detections_list>(BEV_merged_obstacles_topic, 20, true);

	nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_ros_node/pub_rectangular_monitorized_area_marker", rectangular_monitorized_area_marker_topic);
	pub_monitorized_area = nh.advertise<visualization_msgs::Marker>(rectangular_monitorized_area_marker_topic, 20, true);

	// Subscribers

	std::string road_curvature_topic, localization_pose_topic;

	// nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_ros_node/sub_road_curvature", road_curvature_topic);
	// sub_road_curvature = nh.subscribe(road_curvature_topic, 15, road_curvature_cb);

	// nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_ros_node/sub_localization_pose", localization_pose_topic);
	// sub_odom = nh.subscribe(localization_pose_topic, 15, odom_cb);

	message_filters::Subscriber<t4ac_msgs::BEV_detections_list> sub_bev_image_detections;
	message_filters::Subscriber<t4ac_msgs::BEV_detections_list> sub_bev_lidar_detections;

	std::string BEV_image_obstacles_topic, BEV_lidar_obstacles_topic;

	nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_ros_node/sub_BEV_image_obstacles", BEV_image_obstacles_topic);
	sub_bev_image_detections.subscribe(nh, BEV_image_obstacles_topic, 10);

	nh.getParam("/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_ros_node/sub_BEV_lidar_obstacles", BEV_lidar_obstacles_topic);
	sub_bev_lidar_detections.subscribe(nh, BEV_lidar_obstacles_topic, 10);

	// Callback 1: Synchronize LiDAR point cloud based BEV detections and Depth map & 2D Object detectio based BEV detections

	typedef message_filters::sync_policies::ApproximateTime<t4ac_msgs::BEV_detections_list, t4ac_msgs::BEV_detections_list> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub_bev_image_detections, sub_bev_lidar_detections);
	sync.registerCallback(boost::bind(&sensor_fusion_cb, _1, _2));

	ros::spin ();
}

// End Main //


// Definitions of functions //

float euclidean_distance(Point bev_image_detection, Point bev_lidar_detection)
{
	float x_diff = bev_image_detection.get_x()-bev_lidar_detection.get_x();
	float y_diff = bev_image_detection.get_y()-bev_lidar_detection.get_y();
	float ed = float(sqrt(pow(x_diff,2)+pow(y_diff,2)));
	return ed;
}

bool inside_monitorized_area(Point bev_detection, float monitorized_area[])
{
	// std::cout << "BEV image obstacle: " << bev_detection.get_x() << " " << bev_detection.get_y() << std::endl;
	// std::cout << "xmax xmin ymax ymin: " << monitorized_area[0] << " " << monitorized_area[1] << " " << monitorized_area[2] << " " << monitorized_area[3] << std::endl;
	if (bev_detection.get_x() < monitorized_area[0] && bev_detection.get_x() > monitorized_area[1]
		&& bev_detection.get_y() < monitorized_area[2] && bev_detection.get_y() > monitorized_area[3])
		return true;
	else
		return false;
}

// Callbacks

void road_curvature_cb(const std_msgs::Float64::ConstPtr& road_curvature_msg)
{
	road_curvature = road_curvature_msg->data;	
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	float vel_x = odom_msg->twist.twist.linear.x;
	float vel_y = odom_msg->twist.twist.linear.y;
	ego_vel = float(sqrt(pow(vel_x,2)+pow(vel_y,2)));
}

void sensor_fusion_cb(const t4ac_msgs::BEV_detections_list::ConstPtr& bev_image_detections_msg, 
                      const t4ac_msgs::BEV_detections_list::ConstPtr& bev_lidar_detections_msg)
{
	visualization_msgs::MarkerArray merged_obstacles_marker_array;

	t4ac_msgs::BEV_detections_list merged_obstacles;
	merged_obstacles.header = bev_lidar_detections_msg->header;
	// std::cout << "\n\nStamp: " << merged_obstacles.header.stamp.toNSec() << std::endl;
	merged_obstacles.front = bev_lidar_detections_msg->front;
	merged_obstacles.back = bev_lidar_detections_msg->back;
	merged_obstacles.left = bev_lidar_detections_msg->left;
	merged_obstacles.right = bev_lidar_detections_msg->right;

	std::vector<int> evaluated_lidar_obstacles;

	// Define monitorized area

	float xmax, xmin, ymax, ymin, a;
	int id_marker = 0;
	
	float predicted_distance = ego_vel * 2; // The distance 4 seconds ahead

	ratio = road_curvature/max_road_curvature;
	xmax = ratio*predicted_distance;
	if (xmax < 15 && ratio > 0.8)
	{
		xmax = 15;
	}

    xmin = 0;
    ymax = ratio*4.0;
    ymin = ratio*(-4.0);

	visualization_msgs::Marker monitorized_area_marker;

	monitorized_area_marker.header.frame_id = bev_lidar_detections_msg->header.frame_id;
	monitorized_area_marker.type = visualization_msgs::Marker::CUBE;
	// monitorized_area_marker.lifetime = ros::Duration(0.40);
	monitorized_area_marker.pose.position.x = xmax/2;
	monitorized_area_marker.pose.position.y = (ymin+ymax)/2;
	monitorized_area_marker.pose.position.z = -2;
	monitorized_area_marker.scale.x = xmax;
	monitorized_area_marker.scale.y = abs(ymin)+ymax;
	monitorized_area_marker.scale.z = 0.3;
	monitorized_area_marker.color.r = 255;
	monitorized_area_marker.color.g = 255;
	monitorized_area_marker.color.b = 255;
	monitorized_area_marker.color.a = 0.4;

	pub_monitorized_area.publish(monitorized_area_marker);

	//std::cout << "Monitorized area: " << xmax << std::endl;

	float monitorized_area[] = {xmax,xmin,ymax,ymin};

	// Associate bev_detections using GNN (Global Nearest Neighbour) algorithm. TODO: Improve this late fusion
	//std::cout << "Image obstacles: " << bev_image_detections_msg->bev_detections_list.size() << std::endl;
	for (size_t i=0; i<bev_image_detections_msg->bev_detections_list.size(); i++)
	{
		float max_diff = 4; // Initialize maximum allowed difference
		int index_most_similar = -1;

		float x_image, y_image; // LiDAR frame (BEV information based on the depth map)
		x_image = bev_image_detections_msg->bev_detections_list[i].x;
		y_image = bev_image_detections_msg->bev_detections_list[i].y;

		string type = bev_image_detections_msg->bev_detections_list[i].type;
		float score = bev_image_detections_msg->bev_detections_list[i].score;

		Point bev_image_detection(x_image, y_image);

		if (bev_lidar_detections_msg->bev_detections_list.size() > 0)
		{
			float x_closest_lidar, y_closest_lidar;
			x_closest_lidar = y_closest_lidar = 0;

			for (size_t j=0; j<bev_lidar_detections_msg->bev_detections_list.size(); j++)
			{
				// float x_aux = (bev_lidar_detections_msg->bev_detections_list[j].x_corners[2]+
				//                bev_lidar_detections_msg->bev_detections_list[j].x_corners[3]) / 2;
				// x_aux += bev_lidar_detections_msg->bev_detections_list[j].x;

				// float y_aux = (bev_lidar_detections_msg->bev_detections_list[j].y_corners[2]+
				//                bev_lidar_detections_msg->bev_detections_list[j].y_corners[3]) / 2;
				// y_aux += bev_lidar_detections_msg->bev_detections_list[j].y;
				float x_aux = bev_lidar_detections_msg->bev_detections_list[j].x;
				float y_aux = bev_lidar_detections_msg->bev_detections_list[j].y;
				
				Point bev_lidar_detection(x_aux, y_aux);

				float ed = euclidean_distance(bev_image_detection, bev_lidar_detection);

				if (ed < max_diff)
				{
					max_diff = ed;
					index_most_similar = j;
					x_closest_lidar = x_aux;
					y_closest_lidar = y_aux;
				}
			}

			if ((index_most_similar != -1 && 
			    !(std::find(evaluated_lidar_obstacles.begin(), evaluated_lidar_obstacles.end(), index_most_similar) != evaluated_lidar_obstacles.end()))
				|| inside_monitorized_area(bev_image_detection,monitorized_area))
			{
				evaluated_lidar_obstacles.push_back(index_most_similar);

				// Visual marker of merged obstacle

				visualization_msgs::Marker merged_obstacle_marker;

				merged_obstacle_marker.header.frame_id = bev_lidar_detections_msg->header.frame_id;
				merged_obstacle_marker.ns = "merged_obstacles";
				merged_obstacle_marker.id = id_marker;
				id_marker++;
				merged_obstacle_marker.action = visualization_msgs::Marker::ADD;
				merged_obstacle_marker.type = visualization_msgs::Marker::CUBE;
				merged_obstacle_marker.lifetime = ros::Duration(0.2); 

				if (x_closest_lidar != 0 && y_closest_lidar != 0)
				{
					merged_obstacle_marker.pose.position.x = x_closest_lidar;
					merged_obstacle_marker.pose.position.y = y_closest_lidar;
				}
				else
				{
					merged_obstacle_marker.pose.position.x = x_image;
					merged_obstacle_marker.pose.position.y = y_image;
				}
				
				merged_obstacle_marker.pose.position.z = -1.5;
				merged_obstacle_marker.scale.x = 1;
				merged_obstacle_marker.scale.y = 1;
				merged_obstacle_marker.scale.z = 1;
				merged_obstacle_marker.color.r = 0;
				merged_obstacle_marker.color.g = 255;
				merged_obstacle_marker.color.b = 0;
				merged_obstacle_marker.color.a = 0.8;

				// T4AC BEV detection of merged obstacle -> Used by decision-making layer

				t4ac_msgs::BEV_detection merged_obstacle;

				merged_obstacle.type = type;
                merged_obstacle.score = score;

				if (x_closest_lidar != 0 && y_closest_lidar != 0)
				{
					merged_obstacle.x = -y_closest_lidar;
					merged_obstacle.y = -x_closest_lidar;
					merged_obstacle.x_corners = bev_lidar_detections_msg->bev_detections_list[index_most_similar].x_corners;
					merged_obstacle.y_corners = bev_lidar_detections_msg->bev_detections_list[index_most_similar].y_corners;
				}
				else
				{
					merged_obstacle.x = -y_image;
					merged_obstacle.y = -x_image;
					float side = 0.75;
					merged_obstacle.x_corners = {-side,side,-side,side};
					merged_obstacle.y_corners = {-side,-side,side,side};
					merged_obstacle.safety_zone = true;
				}

				merged_obstacles_marker_array.markers.push_back(merged_obstacle_marker);
				merged_obstacles.bev_detections_list.push_back(merged_obstacle);
			}
		}

		
	}

	// Publish merged obstacles

	if (merged_obstacles_marker_array.markers.size() == 0)
	{
		visualization_msgs::Marker merged_obstacle_marker;

		merged_obstacle_marker.header.frame_id = bev_lidar_detections_msg->header.frame_id;
		merged_obstacle_marker.ns = "merged_obstacles";
		merged_obstacle_marker.id = id_marker;
		merged_obstacle_marker.action = visualization_msgs::Marker::ADD;
		merged_obstacle_marker.type = visualization_msgs::Marker::CUBE;
		merged_obstacle_marker.lifetime = ros::Duration(0.2); //ros::Duration(0.01);
		merged_obstacle_marker.pose.position.x = 0;
		merged_obstacle_marker.pose.position.y = 0;
		merged_obstacle_marker.scale.x = 1;
		merged_obstacle_marker.scale.y = 1;
		merged_obstacle_marker.scale.z = 1;
		merged_obstacle_marker.color.r = 0;
		merged_obstacle_marker.color.g = 255;
		merged_obstacle_marker.color.b = 0;
		merged_obstacle_marker.color.a = 0.8;

		merged_obstacles_marker_array.markers.push_back(merged_obstacle_marker);
	}

	// std::cout << "Merged obstacles: " << merged_obstacles_marker_array.markers.size() << std::endl;
	pub_merged_obstacles_marker_array.publish(merged_obstacles_marker_array);
	pub_merged_obstacles.publish(merged_obstacles);
}

// End Definitions of functions //

/*if (inside_monitorized_area(bev_image_detection,monitorized_area))
		{
			// Visual marker of merged obstacle

			visualization_msgs::Marker merged_obstacle_marker;

			merged_obstacle_marker.header.frame_id = bev_lidar_detections_msg->header.frame_id;
			merged_obstacle_marker.ns = "merged_obstacles";
			merged_obstacle_marker.id = index_most_similar;
			merged_obstacle_marker.action = visualization_msgs::Marker::ADD;
			merged_obstacle_marker.type = visualization_msgs::Marker::CUBE;
			merged_obstacle_marker.lifetime = ros::Duration(0.40);

			merged_obstacle_marker.pose.position.x = x_image;
			merged_obstacle_marker.pose.position.y = y_image;
			
			merged_obstacle_marker.pose.position.z = -1.5;
			merged_obstacle_marker.scale.x = 1;
			merged_obstacle_marker.scale.y = 1;
			merged_obstacle_marker.scale.z = 1;
			merged_obstacle_marker.color.r = 0;
			merged_obstacle_marker.color.g = 255;
			merged_obstacle_marker.color.b = 0;
			merged_obstacle_marker.color.a = 0.8;

			// T4AC BEV detection of merged obstacle -> Used by decision-making layer

			t4ac_msgs::BEV_detection merged_obstacle;

			merged_obstacle.type = type;
			merged_obstacle.score = score;

			merged_obstacle.x = -y_image;
			merged_obstacle.y = -x_image;
			float side = 0.75;
			merged_obstacle.x_corners = {-side,side,-side,side};
			merged_obstacle.y_corners = {-side,-side,side,side};
			merged_obstacle.safety_zone = true;

			merged_obstacles_marker_array.markers.push_back(merged_obstacle_marker);
			merged_obstacles.bev_detections_list.push_back(merged_obstacle);
		}*/
