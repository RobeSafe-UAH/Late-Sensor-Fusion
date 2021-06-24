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

# ROS imports

#sys.path.insert(0,'/opt/ros/melodic/lib/python2.7/dist-packages')
import rospy

from message_filters import TimeSynchronizer, ApproximateTimeSynchronizer, Subscriber
from t4ac_msgs.msg import Bounding_Box_3D, Bounding_Box_3D_list
from visualization_msgs.msg import Marker, MarkerArray

root = rospy.get_param('/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensor_fusion_3D_ros_node/root')

# https://github.com/udacity/didi-competition/blob/master/tracklets/python/evaluate_tracklets.py
# https://github.com/AlienCat-K/3D-IoU-Python/blob/master/3D-IoU-Python.py

class Sensor_Fusion_3D():
    """
    """
    def __init__(self):

        # Aux variables

        self.min_overlap = 0.25 # Minimum bounding box overlap for 3D IoU

        # Subscribers

        camera_3D_obstacles_topic = rospy.get_param(os.path.join(root,'sub_3D_camera_obstacles'))
        lidar_3D_obstacles_topic = rospy.get_param(os.path.join(root,'sub_3D_lidar_obstacles'))

        self.sub_camera_3D_obstacles = Subscriber(camera_3D_obstacles_topic, Bounding_Box_3D_list)
        self.sub_lidar_3D_obstacles = Subscriber(lidar_3D_obstacles_topic, Bounding_Box_3D_list)

        header_synchro = 60
        slop = 0.25

        self.ts = ApproximateTimeSynchronizer([self.sub_camera_3D_obstacles,
                                               self.sub_lidar_3D_obstacles],
                                               header_synchro,slop)
        # self.ts = TimeSynchronizer([self.sub_camera_3D_obstacles,
        #                             self.sub_lidar_3D_obstacles],
        #                             header_synchro)
        self.ts.registerCallback(self.sensor_fusion_3d_callback)

        # Publishers

        merged_3D_obstacles_topic = rospy.get_param(os.path.join(root,'pub_3D_merged_obstacles'))
        self.pub_merged_3D_obstacles = rospy.Publisher(merged_3D_obstacles_topic, Bounding_Box_3D_list, queue_size=20)

        merged_3D_obstacles_marker_topic = rospy.get_param(os.path.join(root,'pub_3D_merged_obstacles_marker'))
        self.pub_3D_merged_obstacles_marker = rospy.Publisher(merged_3D_obstacles_marker_topic, MarkerArray, queue_size=20)

    def sensor_fusion_3d_callback(self, camera_3d_obstacles_msg, lidar_3d_obstacles_msg):
        """
        """
        print(">>>>>>>>>>>>>>>>>")
        print("Camera stamp: ", camera_3d_obstacles_msg.header.stamp.to_sec())
        print("LiDAR stamp: ", lidar_3d_obstacles_msg.header.stamp.to_sec())

        # image_msg.header.stamp = pointcloud_msg.header.stamp
        # pub_synchronized_image.publish(image_msg)
        # pub_synchronized_pointcloud.publish(pointcloud_msg)

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