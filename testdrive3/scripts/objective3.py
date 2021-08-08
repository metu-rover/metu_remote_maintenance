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



if __name__ == '__main__':
    try:
        print ("Starting moveit_commander")
        ur3_arm = arm()
        print("Going to start state")
        ur3_arm.go_to_joint_state(ur3_arm.start_state)


        # load poses from markers_precise.csv
        f = open('markers_precise.csv',"r+")
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            print(row)
            ur3_arm.loaded_marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED
            ur3_arm.marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED


	ur3_arm.initialize_environment()

        print ("Starting to scan imu location")
        ur3_arm.t2_scan_imu_module_location()
        ur3_arm.go_to_imu_goal()
        rospy.sleep(1)
        ur3_arm.approach_imu() # added simulation and test drive parts
        rospy.sleep(1)
        ur3_arm.t2_spawn_imu() # added simulation and test drive parts
        rospy.sleep(1)
        ur3_arm.grab_imu()
        rospy.sleep(1)
        ur3_arm.t2_go_to_joint_state()
        rospy.sleep(1)

        print ("============ Objective 3 completed!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
