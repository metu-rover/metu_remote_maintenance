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

        f = open('ids.csv',"r+")
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            print(row)
            ur3_arm.loaded_marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED

        print ("Closing the inspection window")
        ur3_arm.move_to_area()
        rospy.sleep(1)
        ur3_arm.t3_grab_cover()
        rospy.sleep(1)
        ur3_arm.t3_move_back_to_cover()
        rospy.sleep(1)
        ur3_arm.t3_detach_cover()
        rospy.sleep(1)


        print ("============ Objective8 completed!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
