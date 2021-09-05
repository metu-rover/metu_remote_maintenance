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

        rospy.loginfo ("scanning imu location")
        ur3_arm.t2_scan_imu_module_location()


        rospy.loginfo ("going to imu goal")
        ur3_arm.go_to_imu_goal()
        rospy.sleep(1)


        rospy.loginfo ("approaching imu")
        ur3_arm.approach_imu() # added simulation and test drive parts
        rospy.sleep(1)


        rospy.loginfo ("spawning imu")
        ur3_arm.t2_spawn_imu() # added simulation and test drive parts
        rospy.sleep(9)

        
        rospy.loginfo ("grabbing imu")
        ur3_arm.grab_imu()
        rospy.sleep(1)


        rospy.loginfo ("going to designated joint state")
        ur3_arm.t2_go_to_joint_state()
        rospy.sleep(1)


        rospy.loginfo ("Objective 3 Completed !")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
