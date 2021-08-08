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



if __name__ == '__main__':
    try:
        print ("Starting moveit_commander")
        ur3_arm = arm()
        print("Going to start state")
        ur3_arm.go_to_joint_state(ur3_arm.start_state)

        print ("Starting to scan the marker shown in the inspection window")
        ur3_arm.t3_detect_tag()
        rospy.sleep(1)





        rospy.loginfo("Loading panel button ids")
        f1 = open('markers_precise.csv',"r+")
        reader = csv.reader(f1,delimiter=' ')
        for row in reader:
            print(row)
            ur3_arm.loaded_marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED


        rospy.loginfo("Writing id to obj6id.csv")
        f = open("obj6id.csv", 'a')
        f =  open("obj6id.csv", 'w+')
        writer = csv.writer(f,delimiter=' ')
        for marker_id in ur3_arm.button:  
            if marker_id < 10:
                marker_tf = ur3_arm.button[marker_id] 
                x = marker_tf.transform.translation.x
                y = marker_tf.transform.translation.y
                z = marker_tf.transform.translation.z
                if not ur3_arm.check_all_close([x,y,z],ur3_arm.loaded_marker_poses[int(marker_id)],0.001):
                   print([marker_id,x,y,z])
                   writer.writerow([marker_id,x,y,z])


        print ("============ Objective6 completed!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
