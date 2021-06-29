# ROS imports

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import std_msgs.msg
from aux_functions.geometric_functions import euler_from_quaternion

from pyquaternion import Quaternion

def yaw2quaternion(yaw: float) -> Quaternion:
    """
    """
    return Quaternion(axis=[0,0,1], radians=yaw)

def marker_bb(header,frame_id,box,label,id_marker,corners=False):
    """
    If corners = True, visualize the 3D corners instead of a solid cube
    """

    colors_label = [(0,0,255), (255,255,0), (128,0,0), # car(blue), truck(yellow), construction_vehicle(maroon)
                    (0,128,128), (0,128,0), (0,255,255), # bus(teal), trailer(green), barrier(cyan)
                    (0,255,0), (128,128,128), (255,0,255), (128,0,128)] #motorcycle(lime), bicycle(grey), pedestrian(magenta), traffic_cone(purple)
    # print("box merged: ", box)
    print("id marker: ", id_marker)
    box_marker = Marker()
    box_marker.header.stamp = header.stamp
    box_marker.header.frame_id = frame_id
    box_marker.action = Marker.ADD
    box_marker.id = id_marker
    box_marker.lifetime = rospy.Duration.from_sec(0.2)

    box_marker.type = Marker.CUBE
    box_marker.pose.position.x = box[2][0]
    box_marker.pose.position.y = box[2][1]
    box_marker.pose.position.z = box[2][2]
    q = yaw2quaternion(box[1])
    box_marker.pose.orientation.x = q[1] 
    box_marker.pose.orientation.y = q[2]
    box_marker.pose.orientation.z = q[3]
    box_marker.pose.orientation.w = q[0]
    box_marker.scale.x = box[0][2]
    box_marker.scale.y = box[0][1]
    box_marker.scale.z = box[0][0]
    # color_norm = map(lambda x: x/255, colors_label[label-1])
    # box_marker.color.r, box_marker.color.g, box_marker.color.b = color_norm
    box_marker.color.a = 1.0

    return box_marker

def bbros_to_bbtuple(bbros):
    """
    """

    quaternion = [bbros.pose.pose.orientation.x,
                  bbros.pose.pose.orientation.y,
                  bbros.pose.pose.orientation.z,
                  bbros.pose.pose.orientation.w]

    roll, pitch, yaw = euler_from_quaternion(quaternion)

    l = bbros.l
    w = bbros.w
    h = bbros.h

    x = bbros.pose.pose.position.x
    y = bbros.pose.pose.position.y
    z = bbros.pose.pose.position.z

    bbtuple = ((h,w,l),yaw,(x,y,z))

    return bbtuple