#!/usr/bin/env python  

import rospy
import tf
from nav_msgs.msg import Odometry
import geometry_msgs.msg

def tf_to_odom():
    rospy.init_node('tf_2_odom')

    # 1. Parámetros (con valores por defecto para que no falle si no hay launch)
    frequency = rospy.get_param('~frequency', 10.0)
    input_frame1 = rospy.get_param('~input_frame1', '/mussol/nav')
    input_frame2 = rospy.get_param('~input_frame2', '/pose_corrected')
    output_frame = rospy.get_param('~output_frame', 'mussol/nav')
    child_frame = rospy.get_param('~child_frame', 'mussol')

    # Covarianzas
    cv = [rospy.get_param('~covariance_x', 0.1),
          rospy.get_param('~covariance_y', 0.1),
          rospy.get_param('~covariance_z', 0.1),
          rospy.get_param('~covariance_roll', 0.1),
          rospy.get_param('~covariance_pitch', 0.1),
          rospy.get_param('~covariance_yaw', 0.1)]

    listener = tf.TransformListener()
    
    # IMPORTANTE: El nombre del publisher debe coincidir con el que usas abajo
    odom_pub = rospy.Publisher('~odom_corrected', Odometry, queue_size=5)
    
    last_timestamp = rospy.Time(0)
    rate = rospy.Rate(frequency)

    rospy.loginfo("Nodo tf_2_odom iniciado. Escuchando entre %s y %s", input_frame1, input_frame2)

    while not rospy.is_shutdown():
        try:
            # Buscamos la transformación más reciente
            t = rospy.Time(0)
            if listener.canTransform(input_frame1, input_frame2, t):
                (trans, rot) = listener.lookupTransform(input_frame1, input_frame2, t)
                timestamp = listener.getLatestCommonTime(input_frame1, input_frame2)
                
                if timestamp != last_timestamp:
                    msg = Odometry()
                    msg.header.stamp = timestamp
                    msg.header.frame_id = output_frame
                    msg.child_frame_id = child_frame

                    # Pose
                    msg.pose.pose.position.x = trans[0]
                    msg.pose.pose.position.y = trans[1]
                    msg.pose.pose.position.z = trans[2]
                    msg.pose.pose.orientation.x = rot[0]
                    msg.pose.pose.orientation.y = rot[1]
                    msg.pose.pose.orientation.z = rot[2]
                    msg.pose.pose.orientation.w = rot[3]

                    # Llenar diagonal de covarianza
                    msg.pose.covariance[0] = cv[0]
                    msg.pose.covariance[7] = cv[1]
                    msg.pose.covariance[14] = cv[2]
                    msg.pose.covariance[21] = cv[3]
                    msg.pose.covariance[28] = cv[4]
                    msg.pose.covariance[35] = cv[5]

                    odom_pub.publish(msg)
                    last_timestamp = timestamp
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logdebug("TF no disponible todavía: %s", str(e))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        tf_to_odom()
    except rospy.ROSInterruptException:
        pass