#! /usr/bin/env python3.8

"""
Created on Mon Mar 22 13:51:12 2020

@author: Carlos Gómez-Huélamo

Code to 

Communications are based on ROS (Robot Operating Sytem)

Inputs:
Outputs: 

Note that 

Executed via
"""

# General-use imports

import os
import sys
import time
import math

# ROS imports

import rospy
from message_filters import TimeSynchronizer, ApproximateTimeSynchronizer, Subscriber
from t4ac_msgs.msg import Bounding_Box_3D, Bounding_Box_3D_list
from visualization_msgs.msg import Marker, MarkerArray

# 3D IoU

from aux_functions.geometric_functions import euclidean_distance
from aux_functions.iou_3d_functions import compute_box_3d, box3d_iou
from aux_functions.ros_functions import bbros_to_bbtuple, marker_bb

# https://github.com/udacity/didi-competition/blob/master/tracklets/python/evaluate_tracklets.py
# https://github.com/AlienCat-K/3D-IoU-Python/blob/master/3D-IoU-Python.py

# Global variables

root = rospy.get_param('/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_3D_ros_node/root')

class Sensor_Fusion_3D():
    """
    """
    def __init__(self):

        # Aux variables

        self.min_overlap = 0.25 # Minimum bounding box overlap for 3D IoU
        self.prev_time = self.curr_time = 0.0

        # Subscribers

        camera_3D_obstacles_topic = rospy.get_param(os.path.join(root,'sub_3D_camera_obstacles'))
        lidar_3D_obstacles_topic = rospy.get_param(os.path.join(root,'sub_3D_lidar_obstacles'))

        self.sub_camera_3D_obstacles = Subscriber(camera_3D_obstacles_topic, Bounding_Box_3D_list)
        self.sub_lidar_3D_obstacles = Subscriber(lidar_3D_obstacles_topic, Bounding_Box_3D_list)

        header_synchro = 20
        slop = 0.05

        self.ts = ApproximateTimeSynchronizer([self.sub_camera_3D_obstacles,
                                               self.sub_lidar_3D_obstacles],
                                               header_synchro,slop)

        self.ts.registerCallback(self.sensor_fusion_3d_callback)

        # Publishers

        merged_3D_obstacles_topic = rospy.get_param(os.path.join(root,'pub_3D_merged_obstacles'))
        self.pub_merged_3D_obstacles = rospy.Publisher(merged_3D_obstacles_topic, Bounding_Box_3D_list, queue_size=20)

        merged_3D_obstacles_marker_topic = rospy.get_param(os.path.join(root,'pub_3D_merged_obstacles_marker'))
        self.pub_3D_merged_obstacles_marker = rospy.Publisher(merged_3D_obstacles_marker_topic, MarkerArray, queue_size=20)

        self.laser_frame = rospy.get_param('/t4ac/frames/laser')
        self.camera_fov = 85

    def sensor_fusion_3d_callback(self, camera_3d_obstacles_msg, lidar_3d_obstacles_msg):
        """
        """
        print(">>>>>>>>>>>>>>>>")
        self.curr_time = time.time()

        if self.prev_time != 0.0:
            hz = 1 / (self.curr_time-self.prev_time)
            # print("Hz Fusion 3D: ", hz)

        self.prev_time = self.curr_time
        merged_obstacles_marker_array = MarkerArray()

        # print("Camera obstacles: ", len(camera_3d_obstacles_msg.bounding_box_3d_list))
        # print("LiDAR obstacles: ", len(lidar_3d_obstacles_msg.bounding_box_3d_list))

        for cam_obstacle_ros in camera_3d_obstacles_msg.bounding_box_3d_list:
            cam_obstacle_tuple = bbros_to_bbtuple(cam_obstacle_ros)
            cam_3d_corners = compute_box_3d(cam_obstacle_tuple)
            # print("\nCam tuple: ", cam_obstacle_tuple)
            # print("Cam score: ", cam_obstacle_ros.score)
            for i,lidar_obstacle_ros in enumerate(lidar_3d_obstacles_msg.bounding_box_3d_list):
                aux_point_angle = math.atan2(lidar_obstacle_ros.pose.pose.position.x,lidar_obstacle_ros.pose.pose.position.y)
                distance = euclidean_distance(cam_obstacle_ros,lidar_obstacle_ros)

                # print("Distance: ", distance)
                # print("LiDAR score: ", lidar_obstacle_ros.score)
                if lidar_obstacle_ros.type != "Traffic_Cone" and lidar_obstacle_ros.type != "Barrier" and distance < 3:
                    lidar_obstacle_tuple = bbros_to_bbtuple(lidar_obstacle_ros)
                    lidar_3d_corners = compute_box_3d(lidar_obstacle_tuple)
                    # print("LiD tuple: ", lidar_obstacle_tuple)
                    iou3d, _ = box3d_iou(cam_3d_corners,lidar_3d_corners)
                    print("iou3d: ", iou3d)

                    if iou3d > 0.0:
                        box_marker = marker_bb(lidar_3d_obstacles_msg.header,
                                               self.laser_frame,
                                               lidar_obstacle_tuple,
                                               lidar_obstacle_ros.type,
                                               i,corners=False)

                        merged_obstacles_marker_array.markers.append(box_marker)

        self.pub_3D_merged_obstacles_marker.publish(merged_obstacles_marker_array)

def main():
    # Node name

    node_name = rospy.get_param(os.path.join(root,'node_name'))
    rospy.init_node(node_name, anonymous=True)
    Sensor_Fusion_3D()

    try:
        rospy.spin()
    except KeyboardInterruput:
        rospy.loginfo("Shutting down Sensor Fusion 3D module")

if __name__ == '__main__':
    print("Start Sensor Fusion 3D node")
    main()