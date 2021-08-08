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
        print ("Scanning left panel")
        ur3_arm.t2_scan_left_panel()
        ur3_arm.t2_go_to_left_panel()
        rospy.sleep(1)
        ur3_arm.t2_put_on_panel()
        rospy.sleep(1)
        ur3_arm.detach_imu()
        rospy.sleep(1)
        ur3_arm.remove_imu()
        rospy.sleep(1)
        print("Going to start state")
        ur3_arm.go_to_joint_state(ur3_arm.start_state)


        print ("============ objective4 completed!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
