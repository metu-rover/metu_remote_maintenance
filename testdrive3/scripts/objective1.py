#!/usr/bin/python
import sys
import copy
from moveit_commander.planning_scene_interface import PlanningSceneInterface
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
from math import pi, radians
from std_msgs.msg import String
from std_msgs.msg import Bool
from moveit_commander.conversions import pose_to_list
import numpy as np
import tf.transformations
import math as m
import trajectory_msgs.msg as tjmsg
import control_msgs.msg
import actionlib

# Aruco related dependencies
import roslib 
import tf2_ros

from arm_utils import *

import csv



def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])


if __name__ == '__main__':
    try:
        f1 = open("markers.csv", 'a')
        print ("Starting moveit_commander")
        ur3_arm = arm()
        ur3_arm.go_to_joint_state(ur3_arm.start_state)
        print ("Scanning middle panel")
        ur3_arm.t1_scan_middle_panel()
        print("Writing recorded aruco poses to markers.csv")

        f1 = open("markers.csv", 'a')
        f1 =  open("markers.csv", 'w+')
        writer = csv.writer(f1,delimiter=' ')
        for marker_id in ur3_arm.marker_poses:
            marker_tf = ur3_arm.marker_poses[marker_id] 
            x = truncate(marker_tf.transform.translation.x,3)
            y = truncate(marker_tf.transform.translation.y,3)
            z = truncate(marker_tf.transform.translation.z,3)
            print([marker_id,x,y,z])
            writer.writerow([marker_id,x,y,z])


        f2 = open("markers_precise.csv", 'a')
        f2 =  open("markers_precise.csv", 'w+')
        writer = csv.writer(f2,delimiter=' ')
        for marker_id in ur3_arm.marker_poses:
            marker_tf = ur3_arm.marker_poses[marker_id] 
            x = marker_tf.transform.translation.x
            y = marker_tf.transform.translation.y
            z = marker_tf.transform.translation.z
            print([marker_id,x,y,z])
            writer.writerow([marker_id,x,y,z])
        
        print ("============ Objective 1 Completed !")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
