#!/usr/bin/env python  

import rospy
import tf
from nav_msgs.msg import Path
import geometry_msgs.msg 

def tf_to_path():
    rospy.init_node('tf_2_path')

    # 1. Parámetros (con valores por defecto para que no falle si no hay launch)
    frequency = rospy.get_param('~frequency', 10.0)
    ref_frame = rospy.get_param('~ref_frame', '/mussol/nav') # Reference (odom)
    input_frame1 = rospy.get_param('~input_frame1', '/mussol') # Original position
    input_frame2 = rospy.get_param('~input_frame2', '/pose_corrected') # Corrected position
    output_frame = rospy.get_param('~output_frame', 'mussol/nav')


    listener = tf.TransformListener()
    
    path_og_pub = rospy.Publisher('~path_original', Path, queue_size=5)
    og_poses = []
    path_cr_pub = rospy.Publisher('~path_corrected', Path, queue_size=5)
    cr_poses = []
    
    last_timestamp = rospy.Time(0)
    last_timestamp2 = rospy.Time(0)
    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        try:
            # Buscamos la transformación más reciente
            t = rospy.Time(0)
            if listener.canTransform(ref_frame, input_frame1, t):
                (trans, rot) = listener.lookupTransform(ref_frame, input_frame1, t)
                timestamp = listener.getLatestCommonTime(ref_frame, input_frame1)
                
                if timestamp != last_timestamp:

                    msg = Path()
                    msg.header.stamp = timestamp
                    msg.header.frame_id = output_frame

                    # Pose
                    pose = geometry_msgs.msg.PoseStamped()
                    pose.pose.position.x = trans[0]
                    pose.pose.position.y = trans[1]
                    pose.pose.position.z = trans[2]
                    pose.pose.orientation.x = rot[0]
                    pose.pose.orientation.y = rot[1]
                    pose.pose.orientation.z = rot[2]
                    pose.pose.orientation.w = rot[3]

                    og_poses.append(pose)

                    msg.poses = og_poses

                    path_og_pub.publish(msg)
                    last_timestamp = timestamp

            t = rospy.Time(0)
            if listener.canTransform(ref_frame, input_frame2, t):
                (trans, rot) = listener.lookupTransform(ref_frame, input_frame2, t)
                timestamp = listener.getLatestCommonTime(ref_frame, input_frame2)
                
                if timestamp != last_timestamp2:

                    msg = Path()
                    msg.header.stamp = timestamp
                    msg.header.frame_id = output_frame

                    # Pose
                    pose = geometry_msgs.msg.PoseStamped()
                    pose.pose.position.x = trans[0]
                    pose.pose.position.y = trans[1]
                    pose.pose.position.z = trans[2]
                    pose.pose.orientation.x = rot[0]
                    pose.pose.orientation.y = rot[1]
                    pose.pose.orientation.z = rot[2]
                    pose.pose.orientation.w = rot[3]

                    cr_poses.append(pose)

                    msg.poses = cr_poses

                    path_cr_pub.publish(msg)
                    
                    last_timestamp2 = timestamp
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logdebug("TF no disponible todavía: %s", str(e))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        tf_to_path()
    except rospy.ROSInterruptException:
        pass