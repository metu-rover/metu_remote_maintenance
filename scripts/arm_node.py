import sys
import copy
from moveit_commander.planning_scene_interface import PlanningSceneInterface
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
import tf.transformations
import math as m
import trajectory_msgs.msg as tjmsg
import control_msgs.msg
import actionlib



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



def euler_to_quaternion(r,p,yw): 
    quaternion = tf.transformations.quaternion_from_euler(m.radians(r), m.radians(p), m.radians(yw))
    return quaternion


def quaternion_to_euler(quaternion):
    [r,p,yw] = tf.transformations.euler_from_quaternion(quaternion)
    return [r,p,yw]

class arm(object):
  """create arm moveit interface"""
  def __init__(self):
    super(arm, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_action_node', anonymous=True)


    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    #arm group
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #gripper group (do we need it ?)
    gripper_name = "gripper"
    eef_group = moveit_commander.MoveGroupCommander(gripper_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)





    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    # do we need it ?
    self.eef_group = eef_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = move_group.get_planning_frame()
    self.eef_link = move_group.get_end_effector_link()
    self.group_names = robot.get_group_names()











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
    quaternion = euler_to_quaternion(r,p,yw)
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


  def go_to_joint_state(self):
    "move arm to a specified joint state"
    move_group = self.move_group


    current_joints = self.move_group.get_current_joint_values()
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = current_joints[0]
    joint_goal[1] = current_joints[1]
    joint_goal[2] = current_joints[2]
    joint_goal[3] = current_joints[3]
    joint_goal[4] = current_joints[4]
    joint_goal[5] = current_joints[5]


     # go!
    move_group.go(joint_goal, wait=True)

    # safety measures (there is no equivalent clear joint values function)
    move_group.stop()


    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)




  def move_cartesian(self,dx,dy,dz,scale=1):
    " move dx, dy, dz amount using cartesian path"
    move_group = self.move_group
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.x += scale * dx
    wpose.position.y += scale * dy 
    wpose.position.z += scale * dz  # First move up (z)     # wpose.position.y += scale * 0.2  # and sideways (y)


    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    result = move_group.execute(plan, wait=True)
    return result



  def keyboard_teleop(self):
    print("Keyboard teleop")
    print("Basic commands: w s a d z x e -> +x -x +y -y +z -z exit")
    #a = raw_input()
    a = '0'
    while(a != '5'):
      a = raw_input()
      if (a == 'w'):
        print(a)
        result = self.move_cartesian(0.05,0.,0.)
      elif (a == 's'):
        print(a)
        result = self.move_cartesian(-0.05,0.,0.)
      elif (a =='a'):
        print(a)
        result = self.move_cartesian(0,0.05,0.)     
      elif (a =='d'):
        print(a)
        result = self.move_cartesian(0,-0.05,0.) 
      elif (a =='z'):
        print(a)
        result = self.move_cartesian(0,0,0.05)        
      elif (a =='x'):
        print(a)
        result = self.move_cartesian(0,0,-0.05)      
      elif (a == 'r+'):
        print(a)
        current_pose = self.move_group.get_current_pose().pose
        
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z

        quaternion = [current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]
        [r,p,yw] = quaternion_to_euler(quaternion)

        r = m.degrees(r) + 2 # x axis
        p = m.degrees(p)  # z axis
        yw = m.degrees(yw) # y axis

        result = self.go_to_pose_goal(x,y,z,r,p,yw)

      elif (a == 'r-'):
        print(a)
        current_pose = self.move_group.get_current_pose().pose
        
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z

        quaternion = [current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]
        [r,p,yw] = quaternion_to_euler(quaternion)

        r = m.degrees(r) - 2 # x axis
        p = m.degrees(p)  # z axis
        yw = m.degrees(yw) # y axis

        result = self.go_to_pose_goal(x,y,z,r,p,yw)

      elif (a == 'yw+'):
        print(a)
        current_pose = self.move_group.get_current_pose().pose
        
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z

        quaternion = [current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]
        [r,p,yw] = quaternion_to_euler(quaternion)

        r = m.degrees(r) # x axis
        p = m.degrees(p)  + 2 # z axis
        yw = m.degrees(yw) # y axis

        result = self.go_to_pose_goal(x,y,z,r,p,yw)

      elif (a == 'yw-'):
        print(a)
        current_pose = self.move_group.get_current_pose().pose
        
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z

        quaternion = [current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]
        [r,p,yw] = quaternion_to_euler(quaternion)

        r = m.degrees(r) # x axis
        p = m.degrees(p)  - 2 # z axis
        yw = m.degrees(yw) # y axis

        result = self.go_to_pose_goal(x,y,z,r,p,yw)

      elif (a == 'p+'):
        print(a)
        current_pose = self.move_group.get_current_pose().pose
        
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z

        quaternion = [current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]
        [r,p,yw] = quaternion_to_euler(quaternion)

        r = m.degrees(r) # x axis
        p = m.degrees(p) # z axis
        yw = m.degrees(yw)  + 2 # y axis

        result = self.go_to_pose_goal(x,y,z,r,p,yw)


      elif (a == 'p-'):
        print(a)
        current_pose = self.move_group.get_current_pose().pose
        
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z

        quaternion = [current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]
        [r,p,yw] = quaternion_to_euler(quaternion)

        r = m.degrees(r) # x axis
        p = m.degrees(p) # z axis
        yw = m.degrees(yw)  - 2 # y axis

        result = self.go_to_pose_goal(x,y,z,r,p,yw)


      elif (a =='e'):
        print('EXITING NOW')
        break
      else:
        print("Wrong button")
      #while(not rospy.is_shutdown()):


  def plan_cartesian_path(self, scale=1):
    """ specify waypoints to plan cartesian path"""
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group


    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z += scale * 0.1  # First move up (z)     # wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    #wpose.position.x += scale * 0.2
    #waypoints.append(copy.deepcopy(wpose))




    # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    
    return plan, fraction


  def display_trajectory(self, plan):
    "display your plans on rviz"
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher


    ## display trajectory 
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # publish
    display_trajectory_publisher.publish(display_trajectory)


  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)



  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
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






def main():
  try:
    print ("....... Press `Enter` to begin the machine by setting up the moveit_commander ..,...")
    raw_input()
    ur3_arm = arm()
    ur3_arm.keyboard_teleop()
    #print("..... Pose goal demo .....")
    #ur3_arm.go_to_pose_goal(0.3, 0.1, 0.3, 0, 0, 0)
    
    #print("..... Cartesian primites demo .....")
    #result = ur3_arm.move_cartesian(0,0.,0.1)
    #print(result)
    #result = ur3_arm.move_cartesian(0,0.,-0.2)
    #print(result)

    print ("============ Python tutorial demo complete!")
    rospy.sleep(0.01)

  except KeyboardInterrupt:
    return


def pain():
    ur3_arm = arm()
    while True:
        rospy.sleep(0.01)


if __name__ == '__main__':
  main()
  #pain()
