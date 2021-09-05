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

def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])



if __name__ == '__main__':
    try:
        print ("Starting moveit_commander")
        ur3_arm = arm()
        print("Going to start state")
        ur3_arm.go_to_joint_state(ur3_arm.start_state)

        rospy.loginfo("loading poses from markers_precise.csv")
        f = open('markers_precise.csv',"r+")
        reader = csv.reader(f,delimiter=' ')
        for row in reader:
            print(row)
            ur3_arm.loaded_marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED
            ur3_arm.marker_poses[int(row[0])] = [float(row[1]),float(row[2]),float(row[3])] # ADDDDDED


	ur3_arm.initialize_environment()


        ur3_arm.remove_box("lid")
        ur3_arm.remove_box("lid_handle")

        ur3_arm.gripper_pub.publish("open")

        print ("Starting to scan inspection window's location")
        ur3_arm.t3_detect_inspection_panel()
        rospy.sleep(1)

        print("Writing task 3 aruco poses to t3_marker_poses.csv")
        f1 = open("t3_marker_poses.csv", 'a')
        f1 =  open("t3_marker_poses.csv", 'w+')

        f2 = open("markers.csv", 'a')
        
        writer = csv.writer(f1,delimiter=' ')
        writer2 = csv.writer(f2,delimiter=' ')

        for marker_id in ur3_arm.marker_poses:
            if (marker_id == 12 or marker_id == 13 or marker_id == 14):
                marker_tf = ur3_arm.marker_poses[marker_id] 
                x = marker_tf.transform.translation.x
                y = marker_tf.transform.translation.y
                z = marker_tf.transform.translation.z
                print([marker_id,x,y,z])
                writer.writerow([marker_id,x,y,z])
                writer2.writerow([marker_id,truncate(x,3),truncate(y,3),truncate(z,3)])

        ur3_arm.t3_spawn_cover()
        rospy.sleep(1)
        ur3_arm.t3_move_to_cover()
        rospy.sleep(1)
        print ("Grabbing the inspection window cover")
        rospy.sleep(3)

        ur3_arm.t3_grab_cover()
        rospy.sleep(1)
        ur3_arm.t3_go_to_joint_state()
        rospy.sleep(1)
        ur3_arm.move_to_area()
        rospy.sleep(1)
        ur3_arm.t3_detach_cover()
        rospy.sleep(1)









        print ("============ Objective5-part1 completed!")
        rospy.sleep(0.01)

    except KeyboardInterrupt:
        print("OOOPs. got interrupted")
