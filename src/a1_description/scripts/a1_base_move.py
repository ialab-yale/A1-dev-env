#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np 

import time
import tf
import tf2_ros
import geometry_msgs.msg 


if __name__ == '__main__':
    rospy.init_node('a1_tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    rate = rospy.Rate(100)
    for i in range(1000):
        percent = i/1000
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base"
        t.child_frame_id = "trunk"
        t.transform.translation.x = 2 * percent
        t.transform.translation.y = 2 * percent
        t.transform.translation.z = 0
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        rospy.sleep(0.01)
        br.sendTransform(t)
