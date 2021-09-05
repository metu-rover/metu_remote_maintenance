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

#from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# Aruco related dependencies
import roslib
import tf2_ros

from arm_utils import *

import csv


if __name__ == '__main__':
    try:
        rospy.loginfo("initializing arm commander")
        ur3_arm = arm()
        rospy.loginfo("going to start state")
        ur3_arm.go_to_joint_state(ur3_arm.start_state)

        rospy.loginfo("loading poses from markers_precise.csv")
        f = open('markers_precise.csv',"r+")
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            print(row)
            ur3_arm.loaded_marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED
            ur3_arm.marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED


	ur3_arm.initialize_environment()

        rospy.loginfo("scanning left panel")
        ur3_arm.t2_scan_left_panel()


        rospy.loginfo("going to left panel")
        ur3_arm.t2_go_to_left_panel()
        rospy.sleep(1)


        rospy.loginfo("placing imu on left panel")
        ur3_arm.t2_put_on_panel()
        rospy.sleep(1)
    

        rospy.loginfo("detaching imu from arm")
        ur3_arm.detach_imu()
        rospy.sleep(1)


        rospy.loginfo("removing imu from planning space")
        ur3_arm.remove_imu()
        rospy.sleep(1)


        rospy.loginfo("going to start state")
        ur3_arm.go_to_joint_state(ur3_arm.start_state)


        rospy.loginfo("objective 4 completed!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
