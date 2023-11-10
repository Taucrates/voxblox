#!/usr/bin/env python  

import roslib
import rospy
import tf
import geometry_msgs.msg

if __name__ == '__main__':

    rospy.init_node('tf_2_pose')

    frequency = rospy.get_param('frequency', 10.0)

    input_frame1 = rospy.get_param('~input_frame1', '/mussol/nav')
    input_frame2 = rospy.get_param('~input_frame2', '/pose_corrected')
    output_frame = rospy.get_param('~output_frame', 'mussol/nav')

    covariance_x = rospy.get_param('~covariance_x', 0.1)
    covariance_y = rospy.get_param('~covariance_y', 0.1)
    covariance_z = rospy.get_param('~covariance_z', 0.1)
    covariance_roll = rospy.get_param('~covariance_roll', 0.1)
    covariance_pitch = rospy.get_param('~covariance_pitch', 0.1)
    covariance_yaw = rospy.get_param('~covariance_yaw', 0.1)

    listener = tf.TransformListener()

    pose_pub = rospy.Publisher('~pose', geometry_msgs.msg.PoseWithCovarianceStamped,queue_size=1)

    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(input_frame1, input_frame2, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        pose_msg = geometry_msgs.msg.PoseWithCovarianceStamped()

        pose_msg.header.stamp = rospy.get_rostime()
        pose_msg.header.frame_id = output_frame

        pose_msg.pose.pose.position.x = trans[0]
        pose_msg.pose.pose.position.y = trans[1]
        pose_msg.pose.pose.position.z = trans[2]

        pose_msg.pose.pose.orientation.x = rot[0]
        pose_msg.pose.pose.orientation.y = rot[1]
        pose_msg.pose.pose.orientation.z = rot[2]
        pose_msg.pose.pose.orientation.w = rot[3]

        pose_msg.pose.covariance[0] = covariance_x
        pose_msg.pose.covariance[7] = covariance_y
        pose_msg.pose.covariance[14] = covariance_z
        pose_msg.pose.covariance[21] = covariance_roll
        pose_msg.pose.covariance[28] = covariance_pitch
        pose_msg.pose.covariance[35] = covariance_yaw
        
        pose_pub.publish(pose_msg)

        rate.sleep()