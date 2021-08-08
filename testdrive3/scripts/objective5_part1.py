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

        print ("Starting to scan inspection window's location")
        ur3_arm.t3_detect_inspection_panel()
        rospy.sleep(1)

        print("Writing recorded aruco poses to markers.csv")
        f1 = open("ids.csv", 'a')
        f1 =  open("ids.csv", 'w+')
        writer = csv.writer(f1,delimiter=' ')
        for marker_id in ur3_arm.marker_poses:
            marker_tf = ur3_arm.marker_poses[marker_id] 
            x = marker_tf.transform.translation.x
            y = marker_tf.transform.translation.y
            z = marker_tf.transform.translation.z
            print([marker_id,x,y,z])
            writer.writerow([marker_id,x,y,z])

        ur3_arm.t3_spawn_cover()
        rospy.sleep(1)
        ur3_arm.t3_move_to_cover()
        rospy.sleep(1)
        print ("Grabbing the inspection window cover")
        ur3_arm.t3_grab_cover()
        rospy.sleep(1)
        ur3_arm.t3_go_to_joint_state()
        rospy.sleep(1)

        print ("============ Objective5-part1 completed!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
