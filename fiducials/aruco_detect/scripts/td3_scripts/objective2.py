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
        print ("Pressing 4 buttons")
        button_list = rospy.get_param('~buttons')


        # ur3_arm.t2_scan_left_panel()
        # print ("Starting to scan imu location")
        # ur3_arm.t2_scan_imu_module_location()
        # ur3_arm.go_to_imu_goal()
        # rospy.sleep(1)
        # ur3_arm.approach_imu()
        # rospy.sleep(2)
        # ur3_arm.t2_spawn_imu()
        # rospy.sleep(1)
        # ur3_arm.t2_go_to_joint_state()
        # rospy.sleep(1)
        # ur3_arm.t2_put_on_panel()
        # rospy.sleep(1)
        # ur3_arm.detach_imu()
        # rospy.sleep(1)
        # ur3_arm.remove_imu()
        # rospy.sleep(1)


        print ("============ Python tutorial demo complete!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
