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


import csv
import math as m
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
      for index in range(len(goal)):
          if abs(actual[index] - goal[index]) > tolerance:
              return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
      return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
      return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class arm(object):
  """create arm moveit interface"""
  def __init__(self):
    super(arm, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_util_node', anonymous=True)

    
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    #arm group
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #gripper group (do we need it ?)

    # actual competition
    #self.gripper_name = "endeffector"
    # simulation commetn two lines below in competition
    #self.gripper_name = "gripper" # 
    
    #eef_group = moveit_commander.MoveGroupCommander(self.gripper_name) 



    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    # Misc variables
    self.box_name = ''
    self.imu_name = "imu"
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.grasping_group = "manipulator"
    group_names = robot.get_group_names()
    self.group_names = group_names
    #self.eef_group = eef_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = move_group.get_planning_frame()
    self.eef_link = move_group.get_end_effector_link()
    self.group_names = robot.get_group_names()
    self.start_state =  [m.radians(0),m.radians(-120),m.radians(100),m.radians(20),m.radians(90),m.radians(-90)]

    # gripper command publisher
    self.gripper_pub = rospy.Publisher("/gripper_command", String,queue_size=5)

    # aruco marker debugging
    self.marker_poses = {}
    self.detected_markers = []
    self.loaded_marker_poses = {} #ADDDDED

    # button what ?
    self.button = {}
    self.button_id = ''
    self.loaded_button_id = ''





  def check_all_close(self,val1,val2,tolerance):
      return m.sqrt(   m.pow(val1[0]-val2[0],2)  +  m.pow(val1[1]-val2[1],2)  + m.pow(val1[2]-val2[2],2)) < tolerance


  def euler_to_quaternion(self,r,p,yw): 
        quaternion = tf.transformations.quaternion_from_euler(m.radians(r), m.radians(p), m.radians(yw))
        return quaternion


  def quaternion_to_euler(self,quaternion):
        [r,p,yw] = tf.transformations.euler_from_quaternion(quaternion)
        return [r,p,yw]


  def initialize_environment(self):

      button_pose = self.marker_poses[5]
      pose_x = button_pose[0]
      middle_panel = self.spawn_box("middle_panel",pose_x + 0.01 , button_pose[1], button_pose[2] - 0.04, 0, 0, 0, 0.02, 0.3, 0.5)
      left_panel = self.spawn_box("left_panel",pose_x - 0.04 , button_pose[1]+ 0.3, button_pose[2] - 0.04, 0, 0, 18, 0.02, 0.3, 0.5)
      right_panel = self.spawn_box("right_panel",pose_x - 0.04 , button_pose[1] - 0.3, button_pose[2] - 0.21 ,  0, 0,-18, 0.02, 0.3, 0.15)
      stand = self.spawn_box("stand",0 , 0, -0.05, 0, 0, 0, 0.15, 0.15, 0.1)
      connector = self.spawn_box("connector",pose_x/2 + 0.04 , button_pose[1] , -0.05, 0, 0, 0, pose_x/2 + 0.02, 0.15, 0.1)
      table = self.spawn_box("table",0 , 0, -0.12, 0, 0, 0, 0.75, 0.75, 0.02)





  def spawn_box(self,box_name,x,y,z,r,p,yw,dx,dy,dz, timeout=4):

    scene = self.scene  
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"

    orientation = self.euler_to_quaternion(r,p,yw)
    box_pose.pose.orientation.x = orientation[0]
    box_pose.pose.orientation.y = orientation[1]
    box_pose.pose.orientation.z = orientation[2]
    box_pose.pose.orientation.w = orientation[3]

    box_pose.pose.position.x = x  
    box_pose.pose.position.y = y  
    box_pose.pose.position.z = z

    scene.add_box(box_name, box_pose, size=(dx, dy, dz))

    return self.wait_for_state_update(box_name,box_is_known=True, timeout=timeout)


  def wait_for_state_update(self, box_name,box_is_known=False, box_is_attached=False, timeout=4):

    #box_name = box_name#self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


  def attach_box(self,box_name,timeout=4):

    #box_name = #self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    #group_names = self.group_names

    grasping_group = self.grasping_group #self.gripper_name   #"endeffector"

    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

  def detach_box(self, box_name, timeout=4):

    box_name = self.box_name #if name is None else name
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)

    return self.wait_for_state_update(box_name,box_is_known=True, box_is_attached=False, timeout=timeout)  

  def remove_box(self, box_name,timeout=4):

    #box_name = self.box_name
    scene = self.scene
    scene.remove_world_object(box_name)

    return self.wait_for_state_update(box_name,box_is_attached=False, box_is_known=False, timeout=timeout)





# ARUCO MARKER specific functions
  def get_marker_pose(self,marker_id):
        "get transformation (translation and rotations) of specified marker"
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(1.0)
        trans,rot,i = 0, 0, 0
        start = rospy.get_time()
        now = rospy.get_time()
        period = 1
        marker_frame = 'fiducial' + "_" + str(marker_id)
        transform = None
        while now - start < period:
            try:
                transform = tfBuffer.lookup_transform('base_link', marker_frame, rospy.Time(0))
                now = rospy.get_time()

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                now = rospy.get_time()

                rospy.sleep(0.2)
                continue    
        return transform

  def get_all_tfs(self):
        "inquire all available transformations"
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer) # it has to be there else no tfs 
        start = rospy.get_time()
        now = rospy.get_time()
        period = 1
        tfs = 0
        while now - start < period:
            try:
                tfs = tfBuffer._getFrameStrings()
                now = rospy.get_time()

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                now = rospy.get_time()
                rospy.sleep(0.2)
                continue    
        return tfs


# TASK 1 specific functions
  def t1_scan_middle_panel(self):
    rospy.loginfo("Objective 1: Scanning the middle panel to localize components/buttons")

    self.move_group.allow_replanning(True)

    planning_time = 0.5
    self.move_group.set_planning_time(planning_time)
    rospy.loginfo("Planning time set to %s", planning_time)

    planner_1 = "RRTstarkConfigDefault"
    planner_2 = "RRTkConfigDefault"
    self.move_group.set_planner_id(planner_1)
    rospy.loginfo("Planner set to %s",planner_1)

    plan_attempts = 5
    self.move_group.set_num_planning_attempts(plan_attempts)
    rospy.loginfo("Planning attempts set to %s", plan_attempts)

    rospy.loginfo("Starting to scan the panel ...")

    self.t1_extract_aruco_poses()
    rospy.sleep(2)


    rospy.loginfo("First pass")
    self.move_cartesian(0.05, -.05, 0)
    rospy.sleep(2)
    self.t1_extract_aruco_poses()
    rospy.sleep(2)


    self.move_cartesian(0, -.05, 0)
    rospy.sleep(2)
    self.t1_extract_aruco_poses()


    self.move_cartesian(0., -.05, 0)
    rospy.sleep(2)
    self.t1_extract_aruco_poses()




    rospy.loginfo("Second pass")

    self.move_cartesian(0, 0, -0.08)
    rospy.sleep(2)
    self.t1_extract_aruco_poses()
    rospy.sleep(2)


    self.move_cartesian(0, .05, 0)
    rospy.sleep(2)
    self.t1_extract_aruco_poses()


    self.move_cartesian(0., +.05, 0)
    rospy.sleep(2)
    self.t1_extract_aruco_poses()





    rospy.loginfo("Third pass")


    self.move_cartesian(0., -.05, 0)
    rospy.sleep(2)
    self.t1_extract_aruco_poses()
    rospy.sleep(2)


    self.move_cartesian(0, -.05, 0)
    rospy.sleep(2)
    self.t1_extract_aruco_poses()


    self.move_cartesian(0., -.05, 0)
    rospy.sleep(2)
    self.t1_extract_aruco_poses()




  def t1_extract_aruco_poses(self):
        rospy.loginfo("extracting detected aruco marker poses ...")
        tfs =  self.get_all_tfs()
        marker_frames = [marker_frame for marker_frame in tfs if "fiducial_" in marker_frame]
        rospy.loginfo('Detected markers %s', marker_frames)
        for frame in marker_frames:
            marker_id = int(frame.replace('fiducial_','',1))
            marker_pose = self.get_marker_pose(marker_id)
            if marker_id not in self.detected_markers:
                self.detected_markers.append(marker_id)
            self.marker_poses[marker_id] = marker_pose
            rospy.sleep(0.2)

  def t1_press_button(self,button_id): #ADDDDDDED
        rospy.loginfo("pressing button %s", button_id)
        move_group = self.move_group
        marker_pose = self.loaded_marker_poses[button_id] 


        approach_pose = geometry_msgs.msg.Pose()
        approach_pose.position.x = marker_pose[0] - 0.200


        approach_pose.position.y = marker_pose[1]
        print("Y GOAL", approach_pose.position.y )
        approach_pose.position.z = marker_pose[2] -0.05500 #- 0.01

        self.gripper_pub.publish("close")
        rospy.sleep(2)
        current_pose = self.move_group.get_current_pose().pose
        dx = approach_pose.position.x - current_pose.position.x 
        dy = approach_pose.position.y - current_pose.position.y 
        dz = approach_pose.position.z - current_pose.position.z 

        self.move_cartesian(dx, dy, dz)
        rospy.sleep(0.5)


        #test drive
        self.move_cartesian(0.06, 0, 0)
        rospy.sleep(0.5)
        self.move_cartesian(-0.06, 0, 0)
        rospy.sleep(0.5)





  def t1_push_button(self,button_id):
          rospy.loginfo("pressing button %s", button_id)
          move_group = self.move_group
          marker_pose = self.marker_poses[button_id] 


          approach_pose = geometry_msgs.msg.Pose()
          approach_pose.position.x = marker_pose.transform.translation.x - 0.200
          approach_pose.position.y = marker_pose.transform.translation.y 
          print("Y GOAL", approach_pose.position.y )
          approach_pose.position.z = marker_pose.transform.translation.z -0.05500 #- 0.01

          self.gripper_pub.publish("close")
          rospy.sleep(2)
          current_pose = self.move_group.get_current_pose().pose
          dx = approach_pose.position.x - current_pose.position.x 
          dy = approach_pose.position.y - current_pose.position.y 
          dz = approach_pose.position.z - current_pose.position.z 

          self.move_cartesian(dx, dy, dz)
          rospy.sleep(0.5)

          self.move_cartesian(0.06, 0, 0)
          rospy.sleep(0.5)
          self.move_cartesian(-0.06, 0, 0)
          rospy.sleep(0.5)







# TASK 2 specific functions
  def t2_spawn_imu(self):
      #simulation
      #dx, dy, dz = 0.05, 0.165, 0.05
      #test drive
      dx, dy, dz = 0.045, 0.165, 0.04

      imu_marker_pose = self.marker_poses[10]
    
      scene = self.scene

      imu_pose = geometry_msgs.msg.PoseStamped()
      imu_pose.header.frame_id = "base_link"
     
      quaternion = [0,0,0,0]
      [r,p,yw] = self.quaternion_to_euler(quaternion)
    
      r = m.degrees(r)  # x axis
      p = m.degrees(p)  # z axis
      yw = m.degrees(yw) + imu_marker_pose.transform.rotation.y # y axis
      quaternion = self.euler_to_quaternion(r, p, yw)
      imu_pose.pose.orientation.x = quaternion[0]
      imu_pose.pose.orientation.y = quaternion[1]
      imu_pose.pose.orientation.z = quaternion[2]
      imu_pose.pose.orientation.w = quaternion[3]

      imu_pose.pose.position.x = imu_marker_pose.transform.translation.x  
      imu_pose.pose.position.y = imu_marker_pose.transform.translation.y  
      imu_pose.pose.position.z = imu_marker_pose.transform.translation.z - 0.025

      return self.spawn_box(self.imu_name, imu_pose.pose.position.x,imu_pose.pose.position.y,imu_pose.pose.position.z, r,p,yw , dx,dy,dz)

  def t2_scan_imu_module_location(self):
  #  rospy.loginfo("Objective 2 - part 1: Scanning IMU module location to localize IMU")

    rospy.loginfo("opening gripper")
    self.gripper_pub.publish("open")

    scan_pose = [0.153/2,0.75/2-0.02-0.2/2,0.3,90,0,0]   #  [0.0153/2+0.020/2,0.075/2-0.02-0.020/2,0.2,0,90,0]
    #self.move_group.allow_replanning(True)
    
    planning_time = 0.5
    self.move_group.set_planning_time(planning_time)
    rospy.loginfo("Planning time set to %s", planning_time)

    planner_1 = "RRTstarkConfigDefault"
    self.move_group.set_planner_id(planner_1)
    rospy.loginfo("Planner set to %s",planner_1)

    planning_attempts = 5
    self.move_group.set_num_planning_attempts(planning_attempts)
    rospy.loginfo("Planning attempts set to %s", planning_attempts)

    rospy.loginfo("starting to scan the left section [IMU section] ..")

    #for imu with x between 0.075 and 0.12
    rospy.loginfo("going to first pose")
    rospy.sleep(4)

    self.go_to_pose_goal(-0.07, 0.3, 0.12, scan_pose[3], scan_pose[4]+35, scan_pose[5])
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.move_cartesian(0, -0.05, 0)
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.go_to_pose_goal(-0.07, 0.20, 0.1, scan_pose[3], scan_pose[4]+30, scan_pose[5])
    rospy.sleep(2)
    self.t2_extract_aruco_poses()

    #for imu with x between 0.13 and 0.18
    rospy.loginfo("going to second pose")
    rospy.sleep(4)

    self.go_to_pose_goal(-0.03, 0.3, 0.12, scan_pose[3], scan_pose[4]+35, scan_pose[5])
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.move_cartesian(0, -0.05, 0)
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.go_to_pose_goal(-0.03, 0.20, 0.1, scan_pose[3], scan_pose[4]+30, scan_pose[5])
    rospy.sleep(2)
    self.t2_extract_aruco_poses()

    #for imu with x between 0.19 and 0.25
    rospy.loginfo("going to third pose")
    rospy.sleep(4)

    self.go_to_pose_goal(0.05, 0.3, 0.12, scan_pose[3], scan_pose[4]+35, scan_pose[5])
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.move_cartesian(0, -0.05, 0)
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.go_to_pose_goal(0.05, 0.20, 0.1, scan_pose[3], scan_pose[4]+30, scan_pose[5])
    rospy.sleep(2)
    self.t2_extract_aruco_poses()

    #for imu with x between 0.25 and 0.32
    rospy.loginfo("going to fourth pose")
    rospy.sleep(4)

    self.go_to_pose_goal(0.11, 0.3, 0.12, scan_pose[3], scan_pose[4]+35, scan_pose[5])
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.move_cartesian(0, -0.05, 0)
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.move_cartesian(0, -0.05, -0.01)
    rospy.sleep(2)
    self.t2_extract_aruco_poses()

    self.go_to_pose_goal(scan_pose[0], scan_pose[1], scan_pose[2], scan_pose[3]-90, scan_pose[4]+90, scan_pose[5])
    rospy.sleep(2)

    print(self.marker_poses)


  def approach_imu(self):
    marker_pose = self.marker_poses[10]

    move_group = self.move_group
    pose_goal = move_group.get_current_pose().pose
    pose_goal.position.z =   marker_pose.transform.translation.z + 0.11 #-14 + 5/4*3 
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    self.gripper_pub.publish("semi_open")
    rospy.sleep(2)
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)



  def go_to_imu_goal(self):
    rospy.loginfo("Getting close to imu with id %s", 10)
    move_group = self.move_group
    marker_pose = self.marker_poses[10]

    
    approach_pose = geometry_msgs.msg.Pose()
    #approach_pose.position.x = 0.21
    approach_pose.position.x = marker_pose.transform.translation.x
    approach_pose.position.y = marker_pose.transform.translation.y 
    approach_pose.position.z = marker_pose.transform.translation.z + 0.17

    current_pose = self.move_group.get_current_pose().pose     
    quaternion = [current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]
    [r,p,yw] = self.quaternion_to_euler(quaternion)

    r = m.degrees(r)   # x axis
    p = m.degrees(p)  # z axis
    yw = m.degrees(yw) + marker_pose.transform.rotation.y # y axis

    quaternion = self.euler_to_quaternion(r,p,yw)
    approach_pose.orientation.x = quaternion[0]
    approach_pose.orientation.y = quaternion[1]
    approach_pose.orientation.z = quaternion[2]
    approach_pose.orientation.w = quaternion[3]
                      
    move_group.set_pose_target(approach_pose)
    plan = move_group.go(wait=True)

    self.move_cartesian(0,0,0.1)

    move_group.stop()
    move_group.clear_pose_targets()
    


    current_pose = self.move_group.get_current_pose().pose
    return all_close(approach_pose, current_pose, 0.01)


  def grab_imu(self):
    rospy.loginfo("Objective 2 - part 2: Grabbing detected IMu Module")
    self.attach_box("imu")


  def detach_imu(self):
    rospy.loginfo("Detaching imu")
    self.gripper_pub.publish("open")
    rospy.sleep(2)
    self.detach_box("imu")

  def remove_imu(self):
    rospy.loginfo("Removing imu")
    self.remove_box("imu")


  def t2_scan_left_panel(self):
    rospy.loginfo("Objective 1: Scanning the left panel to localize components/buttons")

    self.move_group.allow_replanning(True)

    planning_time = 0.5
    self.move_group.set_planning_time(planning_time)
    rospy.loginfo("Planning time set to %s", planning_time)

    planner_1 = "RRTstarkConfigDefault"
    self.move_group.set_planner_id(planner_1)
    rospy.loginfo("Planner set to %s",planner_1)

    plan_attempts = 7
    self.move_group.set_num_planning_attempts(plan_attempts)
    rospy.loginfo("Planning attempts set to %s", plan_attempts)

    rospy.loginfo("Starting to scan the panel ...")

    self.go_to_pose_goal(0.0, 0.15, 0.45, 0, 0, 0)
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.go_to_pose_goal(-0.1, 0.3, 0.3, 20, 0, 18)
    rospy.sleep(2)
    self.t2_extract_aruco_poses()
    self.go_to_pose_goal(0.1, 0.25 ,0.35, 0, 0, 18)
    rospy.sleep(2)
    self.t2_extract_aruco_poses()

    print(self.marker_poses)


  def t2_go_to_joint_state(self):

    move_group = self.move_group

    self.move_cartesian(0, 0, 0.05)
    rospy.sleep(2)
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 1.57
    joint_goal[1] = -1.57
    joint_goal[2] = 1.30
    joint_goal[3] = -1.57
    joint_goal[4] = -1.57
    joint_goal[5] = 0

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    print("Current pose", move_group.get_current_pose().pose)
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def t2_go_to_left_panel(self):

    move_group = self.move_group
    
    pose_goal = geometry_msgs.msg.Pose() 
   
    pose_goal.position.x = 0.05 
    pose_goal.position.y = 0.2 
    pose_goal.position.z = 0.3

    quaternion = self.euler_to_quaternion(0, 0, 18)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
                      
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)



  def t2_put_on_panel(self):

    rospy.loginfo("Getting close to marker with id %s", 11)
    move_group = self.move_group
    marker_pose = self.marker_poses[11]
    
    pose_goal = geometry_msgs.msg.Pose() 
   
    pose_goal.position.x = marker_pose.transform.translation.x - 0.34
    pose_goal.position.y = marker_pose.transform.translation.y - 0.12
    pose_goal.position.z = marker_pose.transform.translation.z - 0.1

    r = rospy.get_param("/angle")
    r = -r

    quaternion = self.euler_to_quaternion(r, 0, 18)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
                      
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    self.move_cartesian(1.5, 0, 0)
    rospy.loginfo("further")
    self.move_cartesian(0.4, 0, 0)
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def t2_extract_aruco_poses(self):
        rospy.loginfo("extracting detected aruco marker poses ...")
        tfs =  self.get_all_tfs()
        marker_frames = [marker_frame for marker_frame in tfs if "fiducial_" in marker_frame]
        rospy.loginfo('Detected markers %s', marker_frames)
        for frame in marker_frames:
            marker_id = int(frame.replace('fiducial_','',1))
            marker_pose = self.get_marker_pose(marker_id)
            if marker_id not in self.detected_markers:
                self.detected_markers.append(marker_id)
            self.marker_poses[marker_id] = marker_pose
            rospy.sleep(0.2)

            
# TASK 3 specific functions
  def t3_detect_inspection_panel(self):
    rospy.loginfo("Objective 5 - part 1: scanning right panel to localize inspection panel")
    scan_pose = [0.2,-0.15,0.15,90,0,0]   

    self.move_group.allow_replanning(True)

    planning_time = 0.5
    self.move_group.set_planning_time(planning_time)
    rospy.loginfo("Planning time set to %s", planning_time)

    planner_1 = "RRTstarkConfigDefault"
    self.move_group.set_planner_id(planner_1)
    rospy.loginfo("Planner set to %s",planner_1)

    plan_attempts = 5
    self.move_group.set_num_planning_attempts(plan_attempts)
    rospy.loginfo("Planning attempts set to %s", plan_attempts)





    rospy.loginfo("Starting to scan the panel ...")
    self.go_to_pose_goal(0.15, -0.2, 0.15, scan_pose[3], scan_pose[4], scan_pose[5]-18)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.move_cartesian(-0.03, -0.05, -0.02)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.move_cartesian(0.03, 0.05, 0)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.go_to_pose_goal(0.15, -0.2, 0.1, scan_pose[3], scan_pose[4]-20, scan_pose[5]-18)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.move_cartesian(0, -0.02, -0.02)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.go_to_pose_goal(-0.1, -0.3, 0.35, 0, 90,-45)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.go_to_pose_goal(-0.12, -0.3, 0.35, 0, 90,-90)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.go_to_pose_goal(0.15, -0.3, 0.35, 0, 90, 18)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()

    self.move_cartesian(0.1, 0.1, 0)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.move_cartesian(-0.02, -0.05, 0)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()


  
    print(self.marker_poses)
    

  def t3_extract_aruco_poses(self):
        rospy.loginfo("extracting detected aruco marker poses ...")
        tfs =  self.get_all_tfs()
        marker_frames = [marker_frame for marker_frame in tfs if "fiducial_" in marker_frame]
        rospy.loginfo('Detected markers %s', marker_frames)
        for frame in marker_frames:
            marker_id = int(frame.replace('fiducial_','',1))
            marker_pose = self.get_marker_pose(marker_id)
            if marker_id not in self.detected_markers:
                print("added " + "marker " + str(marker_id))
                self.detected_markers.append(marker_id)
            self.marker_poses[marker_id] = marker_pose
            rospy.sleep(0.2)

  def t3_spawn_cover(self):
      marker_pose_12 = self.marker_poses[12]
      pose_x = marker_pose_12.transform.translation.x
      pose_y = marker_pose_12.transform.translation.y
      pose_z = marker_pose_12.transform.translation.z
      print("spawning handle")
      lid = self.spawn_box("lid", pose_x+0.0415, pose_y-0.0135, pose_z+0.03, 0, 0, -18, 0.0996,0.1496, 0.004)
      height = pose_z+0.035+0.013
      lid_handle = self.spawn_box("lid_handle", pose_x+0.0415, pose_y-0.0135, height, 0, 0, -18, 0.035, 0.035, 0.035)

  def t3_move_to_cover(self):
    marker_pose_12 = self.marker_poses[12]
    #marker_pose_13 = self.marker_poses[13]
    move_group = self.move_group

    self.move_cartesian(0.05, -0.05, 0.1)
    approach_pose = geometry_msgs.msg.Pose()
    approach_pose.position.x = marker_pose_12.transform.translation.x + 0.0415
    approach_pose.position.y = marker_pose_12.transform.translation.y - 0.0135
    approach_pose.position.z = marker_pose_12.transform.translation.z + 0.23


    quaternion = self.euler_to_quaternion(90,90,-18)
    approach_pose.orientation.x = quaternion[0]
    approach_pose.orientation.y = quaternion[1]
    approach_pose.orientation.z = quaternion[2]
    approach_pose.orientation.w = quaternion[3]


    move_group.set_pose_target(approach_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    self.move_cartesian(0, 0, -0.05)
    current_pose = self.move_group.get_current_pose().pose
    return all_close(approach_pose, current_pose, 0.01)

  def t3_grab_cover(self):
        rospy.loginfo("Objective 5 - part 2: grabbing inspection window cover in real and planning environment")

        rospy.loginfo("grabbing/attaching inspection window cover in real environment")
        self.gripper_pub.publish("semi_close")
        rospy.sleep(4)
        rospy.loginfo("grabbing/attaching inspection window cover in planning environment")
        self.attach_box("lid")
        self.attach_box("lid_handle")

  def t3_go_to_joint_state(self):

    move_group = self.move_group

    self.move_cartesian(0, 0, 0.05)
    rospy.sleep(2)
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 1.57
    joint_goal[1] = -1.57
    joint_goal[2] = -1.30
    joint_goal[3] = 1.57
    joint_goal[4] = -1.57
    joint_goal[5] = 0

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    print("Current pose", move_group.get_current_pose().pose)
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def t3_scan_cover_storage(self):

    move_group = self.move_group
    
    self.move_group.allow_replanning(True)

    planning_time = 0.5
    self.move_group.set_planning_time(planning_time)
    rospy.loginfo("Planning time set to %s", planning_time)

    planner_1 = "RRTstarkConfigDefault"
    planner_2 = "RRTkConfigDefault"
    self.move_group.set_planner_id(planner_1)
    rospy.loginfo("Planner set to %s",planner_1)

    plan_attempts = 5
    self.move_group.set_num_planning_attempts(plan_attempts)
    rospy.loginfo("Planning attempts set to %s", plan_attempts)

    rospy.loginfo("Starting to scan the panel ...")

    self.go_to_pose_goal(-0.1, -0.3, 0.35, 0, 90,-45)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.go_to_pose_goal(-0.1, -0.3, 0.35, 0, 90,-90)
    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.move_cartesian(0.1, -0.1, 0)
    #rospy.sleep(2)
    #self.t3_extract_aruco_poses()
    print(self.marker_poses)

  def move_to_area(self): 
    marker_pose_14 = self.marker_poses[14]
    move_group = self.move_group

    planning_time = 7
    self.move_group.set_planning_time(planning_time)
    rospy.loginfo("Planning time set to %s", planning_time)

    planner_1 = "RRTstarkConfigDefault"
    planner_2 = "RRTkConfigDefault"
    self.move_group.set_planner_id(planner_1)
    rospy.loginfo("Planner set to %s",planner_1)

    plan_attempts = 10
    self.move_group.set_num_planning_attempts(plan_attempts)
    rospy.loginfo("Planning attempts set to %s", plan_attempts)



    approach_pose = geometry_msgs.msg.Pose()
    approach_pose.position.x = marker_pose_14.transform.translation.x - 0.1
    approach_pose.position.y = marker_pose_14.transform.translation.y - 0.1
    approach_pose.position.z = marker_pose_14.transform.translation.z + 0.15

    quaternion = self.euler_to_quaternion(0,90,0)
    approach_pose.orientation.x = quaternion[0]
    approach_pose.orientation.y = quaternion[1]
    approach_pose.orientation.z = quaternion[2]
    approach_pose.orientation.w = quaternion[3]

    move_group.set_pose_target(approach_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(approach_pose, current_pose, 0.01)



  def move_to_lid_area(self): 
    marker_pose_14 = self.marker_poses[14]
    move_group = self.move_group

    approach_pose = geometry_msgs.msg.Pose()
    approach_pose.position.x = marker_pose_14[0] - 0.1
    approach_pose.position.y = marker_pose_14[1] - 0.1
    approach_pose.position.z = marker_pose_14[2] + 0.18

    quaternion = self.euler_to_quaternion(0,90,0)
    approach_pose.orientation.x = quaternion[0]
    approach_pose.orientation.y = quaternion[1]
    approach_pose.orientation.z = quaternion[2]
    approach_pose.orientation.w = quaternion[3]

    move_group.set_pose_target(approach_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()




    current_pose = self.move_group.get_current_pose().pose
    return all_close(approach_pose, current_pose, 0.01)




  def t3_detach_cover(self):
        rospy.loginfo("detaching inspection window cover in real environment")
        self.gripper_pub.publish("open")
        rospy.sleep(2)
        self.detach_box("lid")
        self.detach_box("lid_handle")

  def t3_detect_tag(self):
   #rospy.loginfo("Objective 5 - part 1: scanning right panel to localize inspection panel")
    scan_pose = [0.2,-0.15,0.15,90,0,0]

    self.move_group.allow_replanning(True)

    planning_time = 0.5
    self.move_group.set_planning_time(planning_time)
    rospy.loginfo("Planning time set to %s", planning_time)

    planner_1 = "RRTstarkConfigDefault"
    planner_2 = "RRTkConfigDefault"
    self.move_group.set_planner_id(planner_1)
    rospy.loginfo("Planner set to %s",planner_1)

    plan_attempts = 5
    self.move_group.set_num_planning_attempts(plan_attempts)
    rospy.loginfo("Planning attempts set to %s", plan_attempts)

    rospy.loginfo("Starting to scan the tag ...")
    self.go_to_pose_goal(0.18, -0.23, 0.35, scan_pose[3], scan_pose[4]+90, scan_pose[5]-18)
    rospy.sleep(2)
    #self.t3_extract_aruco_poses()
    self.go_to_pose_goal(0.37, -0.23, 0.35, scan_pose[3], scan_pose[4]+100, scan_pose[5]-18)
    rospy.sleep(2)
    self.extract_button_id()
    self.move_cartesian(0.08, -0.03, 0.03)
    rospy.sleep(2)
    self.extract_button_id()
    self.move_cartesian(0.03, -0.01, 0)
    rospy.sleep(2)
    self.extract_button_id()
    self.move_cartesian(-0.03, 0.01, 0)
    rospy.sleep(2)
    self.extract_button_id()
    self.move_cartesian(-0.02, -0.02, 0)
    rospy.sleep(2)
    self.extract_button_id()
    self.move_cartesian(0.02, 0.02, 0)
    rospy.sleep(2)
    self.extract_button_id()

    print(self.marker_poses)


  def extract_button_id(self):
        rospy.loginfo("extracting detected aruco marker poses ...")
        tfs =  self.get_all_tfs()
        marker_frames = [marker_frame for marker_frame in tfs if "fiducial_" in marker_frame]
        rospy.loginfo('Detected markers %s', marker_frames)
        for frame in marker_frames:
            marker_id = int(frame.replace('fiducial_','',1))
            self.button_id = int(frame.replace('fiducial_','',1))
            marker_pose = self.get_marker_pose(marker_id)
            if marker_id not in self.detected_markers:
                self.detected_markers.append(marker_id)
            self.button[marker_id] = marker_pose
            rospy.sleep(0.2)

  def t3_move_back_to_cover(self):
    marker_pose_12 = self.marker_poses[12]
    marker_pose_13 = self.marker_poses[13]
    move_group = self.move_group

    self.move_cartesian(0, 0, 0.3)
    rospy.sleep(2)
    self.go_to_pose_goal(0.15, -0.3, 0.35, 90, 90, 0)
    rospy.sleep(2)
    self.go_to_pose_goal(0.15, -0.3, 0.35, 90, 90, -18)
    rospy.sleep(2)
    approach_pose = geometry_msgs.msg.Pose()
    approach_pose.position.x = marker_pose_12.transform.translation.x + 0.0415
    approach_pose.position.y = marker_pose_12.transform.translation.y - 0.0135
    approach_pose.position.z = marker_pose_12.transform.translation.z + 0.27


    quaternion = self.euler_to_quaternion(90,90,-18)
    approach_pose.orientation.x = quaternion[0]
    approach_pose.orientation.y = quaternion[1]
    approach_pose.orientation.z = quaternion[2]
    approach_pose.orientation.w = quaternion[3]


    move_group.set_pose_target(approach_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    self.move_cartesian(0, 0, -0.08)
    current_pose = self.move_group.get_current_pose().pose
    return all_close(approach_pose, current_pose, 0.01)

  def t3_scan_middle_panel(self):
    rospy.loginfo("Objective 1: Scanning the middle panel to localize components/buttons")
    scan_pose = [-0.14,0,0.4,90,0,0]   #  [0.0153/2+0.020/2,0.075/2-0.02-0.020/2,0.2,0,90,0]

    self.move_group.allow_replanning(True)

    planning_time = 0.5
    self.move_group.set_planning_time(planning_time)
    rospy.loginfo("Planning time set to %s", planning_time)

    planner_1 = "RRTstarkConfigDefault"
    planner_2 = "RRTkConfigDefault"
    self.move_group.set_planner_id(planner_1)
    rospy.loginfo("Planner set to %s",planner_1)

    plan_attempts = 5
    self.move_group.set_num_planning_attempts(plan_attempts)
    rospy.loginfo("Planning attempts set to %s", plan_attempts)

    rospy.loginfo("Starting to scan the panel ...")

    #self.go_to_pose_goal(scan_pose[0], scan_pose[1], scan_pose[2], scan_pose[3], scan_pose[4], scan_pose[5])
    #rospy.sleep(2)

    #self.t3_extract_aruco_poses()
    self.go_to_joint_state(self.start_state)
    rospy.sleep(2)
    self.go_to_pose_goal(0.22, 0.01,0.38, scan_pose[3], scan_pose[4], scan_pose[5])
    rospy.sleep(2)
    self.t3_extract_aruco_poses()

    self.move_cartesian(0, -.02, 0)
    #self.go_to_pose_goal(0.22, -0.01,0.38, scan_pose[3], scan_pose[4], scan_pose[5])
    rospy.sleep(2)
    self.t3_extract_aruco_poses()

    self.move_cartesian(0, 0.02, -0.08)


    #self.go_to_pose_goal(0.22, 0.01, 0.30, scan_pose[3], scan_pose[4], scan_pose[5])

    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.move_cartesian(0, -0.02, 0)

    #self.go_to_pose_goal(0.22, -0.01, 0.30, scan_pose[3], scCPUsan_pose[4], scan_pose[5])

    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.move_cartesian(0, 0.02, -0.08)

    #self.go_to_pose_goal(0.22, 0.01, 0.22, scan_pose[3], scan_pose[4], scan_pose[5])

    rospy.sleep(2)
    self.t3_extract_aruco_poses()
    self.move_cartesian(0, -0.02, 0)

    #self.go_to_pose_goal(0.22, -0.01, 0.22, scan_pose[3], scan_pose[4], scan_pose[5])

    rospy.sleep(2)
    self.t3_extract_aruco_poses()

  def t3_push_button(self):

        rospy.loginfo("pressing button %s", 5)
        move_group = self.move_group
        marker_pose = self.marker_poses[5]
        #marker_pose = self.button[self.button_id] 


        approach_pose = geometry_msgs.msg.Pose()
        approach_pose.position.x = marker_pose.transform.translation.x - 0.200
        approach_pose.position.y = marker_pose.transform.translation.y 
        print("Y GOAL", approach_pose.position.y )
        approach_pose.position.z = marker_pose.transform.translation.z -0.05500 #- 0.01

        self.gripper_pub.publish("close")
        rospy.sleep(2)
        current_pose = self.move_group.get_current_pose().pose
        dx = approach_pose.position.x - current_pose.position.x 
        dy = approach_pose.position.y - current_pose.position.y 
        dz = approach_pose.position.z - current_pose.position.z 

        self.move_cartesian(dx, dy, dz)
        rospy.sleep(0.5)
        self.move_cartesian(0.12, 0, 0)
        rospy.sleep(0.5)
        self.move_cartesian(-0.12, 0, 0)
        rospy.sleep(0.5)
        self.gripper_pub.publish("open")
        rospy.sleep(2)



# Motion related functions
  def go_to_pose_goal(self,x,y,z,r,p,yw): 
    """ move arm to specified position wrt base link """

    move_group = self.move_group

    #define pose goal
    pose_goal = geometry_msgs.msg.Pose()

    #position
    pose_goal.position.x = x 
    pose_goal.position.y = y 
    pose_goal.position.z = z 
    
    #orientation
    quaternion = self.euler_to_quaternion(r,p,yw)
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    #set goal,plan and go!
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    
    #safety measures (stop arm and clear goal)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose

    return all_close(pose_goal, current_pose, 0.0001)


  def go_to_joint_state(self,joint_state):
    "move arm to a specified joint state"
    move_group = self.move_group


    current_joints = self.move_group.get_current_joint_values()
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = joint_state[0] #current_joints[0]
    joint_goal[1] = joint_state[1] #current_joints[1]
    joint_goal[2] = joint_state[2] #current_joints[2]
    joint_goal[3] = joint_state[3] #current_joints[3]
    joint_goal[4] = joint_state[4] #current_joints[4]
    joint_goal[5] = joint_state[5] #current_joints[5]


     # go!
    move_group.go(joint_goal, wait=True)

    # safety measures (there is no equivalent clear joint values function)
    move_group.stop()


    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def move_cartesian(self,dx,dy,dz,scale=1):
    " move dx, dy, dz amount using cartesian path"
    move_group = self.move_group
    move_group.set_goal_position_tolerance(0.0001)
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.x += scale * dx
    wpose.position.y += scale * dy 
    wpose.position.z += scale * dz  # First move up (z)     # wpose.position.y += scale * 0.2  # and sideways (y)


    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.0001,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    result = move_group.execute(plan, wait=True)
    return result

# Teleoperation Mode





# if __name__ == '__main__':
#     try:
#         print ("....... Press `Enter` to begin the machine by setting up the moveit_commander ..,...")
#         raw_input()
#         ur3_arm = arm()
#         print("============ Going to start state ...")
#         ur3_arm.go_to_joint_state(ur3_arm.start_state)
#         print("============ Press `Enter` to to move to IMU module scan location ...")
#         raw_input()
#         ur3_arm.t1_scan_middle_panel()
#         print ("============ Python tutorial demo complete!")
#         rospy.sleep(0.01)

#     except KeyboardInterrupt:
#         print("OOOPs. got interrupted")
