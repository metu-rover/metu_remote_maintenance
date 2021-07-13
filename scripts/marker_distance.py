#!/usr/bin/env python  
import roslib
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import tf.transformations


def get_marker_pose(marker_id):
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

def get_all_tfs():
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



if __name__ == '__main__':
    rospy.init_node('marker_node')


    markers = {}



    while(not rospy.is_shutdown()):
        tfs =  get_all_tfs()
        marker_frames = [marker_frame for marker_frame in tfs if "fiducial_" in marker_frame]
       # print("CURRENT AVAILABLE FRAMES")
       # print(marker_frames)
        for frame in marker_frames:
            marker_idd = int(frame.replace('fiducial_','',1))
            marker_pose = get_marker_pose(marker_idd)
            
            if marker_pose is not None:
       #         print("MARK ANTONY")
       #         print(marker_idd)
       #         print(marker_pose)
                markers[marker_idd] = marker_pose

     #   print("DEAR MARKERS")
     #   print(markers)
        for marker_id in markers:
            print("Currently saved markers")
      #      print("NOT MARK ANTONY")
      #      print(marker_id)
      #      print(markers[marker_id])
            marker_pose = markers[marker_id]
            print("ID: ", marker_id)
            print("Pose: ", marker_pose)
        rospy.sleep(5) #making this one 5 and other one lower values made it work and plus remove None's otherwise some timing issue always arose
    







