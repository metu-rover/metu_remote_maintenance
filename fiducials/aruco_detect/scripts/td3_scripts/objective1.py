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



if __name__ == '__main__':
    try:
        print ("Starting moveit_commander")
        ur3_arm = arm()
        ur3_arm.go_to_joint_state(ur3_arm.start_state)
        print ("Scanning middle panel")
        ur3_arm.t1_scan_middle_panel()



        print ("============ Python tutorial demo complete!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
