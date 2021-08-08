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
        ur3_arm.go_to_joint_state(ur3_arm.start_state)
        button_list = rospy.get_param('/buttons')

        button1_id = button_list[0]
        button2_id = button_list[2]
        button3_id = button_list[4]
        button4_id = button_list[6]



	ur3_arm.initialize_environment()

        # load poses from markers_precise.csv
        f = open('markers_precise.csv',"r+")
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            print(row)
            ur3_arm.loaded_marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED
            ur3_arm.marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED


	ur3_arm.initialize_environment()

        print ("Pressing 4 buttons", button_list)

        print("Pressing button id:", button1_id) # need to add a bit more push (to actually trigger it)
        ur3_arm.t1_press_button(int(button1_id))


        print("Pressing button id:", button2_id)
        ur3_arm.t1_press_button(int(button2_id))


        print("Pressing button id:", button3_id)
        ur3_arm.t1_press_button(int(button3_id))


        print("Pressing button id:", button4_id)
        ur3_arm.t1_press_button(int(button4_id))


        print ("============ Objective 2 Completed!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
