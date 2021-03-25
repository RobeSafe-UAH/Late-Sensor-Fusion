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

from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from message_filters import TimeSynchronizer, ApproximateTimeSynchronizer, Subscriber

def sensors_synchro_callback(image_msg,pointcloud_msg):
    """
    """

    print("Image stamp: ", image_msg.header.stamp.to_sec())
    print("PointCloud stamp: ", pointcloud_msg.header.stamp.to_sec())

    image_msg.header.stamp = pointcloud_msg.header.stamp
    pub_synchronized_image.publish(image_msg)
    pub_synchronized_pointcloud.publish(pointcloud_msg)

if __name__ == '__main__':
    rospy.init_node("t4ac_sensors_synchro_ros_node")

    # ROS publishers

    synchronized_image_topic = rospy.get_param('/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensors_synchro_ros_node/pub_synchronized_image')
    synchronized_pointcloud_topic = rospy.get_param('/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensors_synchro_ros_node/pub_synchronized_pointcloud')

    pub_synchronized_image = rospy.Publisher(synchronized_image_topic, Image, queue_size = 20)
    pub_synchronized_pointcloud = rospy.Publisher(synchronized_pointcloud_topic, PointCloud2, queue_size = 20)

    # ROS subscribers

    input_image_topic = rospy.get_param('/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensors_synchro_ros_node/sub_input_image')
    input_pointcloud_topic = rospy.get_param('/t4ac/perception/detection/sensor_fusion/t4ac_sensor_fusion_ros/t4ac_sensors_synchro_ros_node/sub_input_pointcloud')

    sub_input_image = Subscriber(input_image_topic, Image)
    sub_input_pointcloud = Subscriber(input_pointcloud_topic, PointCloud2)

    header_synchro = 100
    slop = 0.1

    ts = ApproximateTimeSynchronizer([sub_input_image,  
                                      sub_input_pointcloud],
                                      header_synchro,slop)

    ts.registerCallback(sensors_synchro_callback)

    rospy.spin()