#!/usr/bin/env python
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import math

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        br.sendTransform((1,0,0.5),
                tf.transformations.quaternion_from_euler(-math.pi / 2,0,0),
                #tf.transformations.quaternion_from_euler(0,0,0),
                #(0,0,0,1),
                rospy.Time.now(),
                'usb_cam',
                'base_laser_front_link')
        #br.sendTransform((0,1,0),
                #tf.transformations.quaternion_from_euler(0,0,0),
                ##(0,0,0,1),
                #rospy.Time.now(),
                #'tf2',
                #'world')
        rate.sleep()
