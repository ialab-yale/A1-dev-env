#!/usr/bin/env python

import rospy 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf
import math
from tf.transformations import quaternion_from_euler
import numpy as np 
import time 


model_state_pub = ModelState()
model_state_pub.model_name = "a1_gazebo"
pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size=1000)
rospy.init_node('move_publisher')

def moveBasePosition(Curr_Pos, targetPos, duration):
    for i in range(duration):
        if rospy.is_shutdown():
            break
        percent = i/duration
        model_state_pub.pose.position.x = Curr_Pos[0]*(1-percent) + targetPos[0]*percent
        model_state_pub.pose.position.y = Curr_Pos[1]*(1-percent) + targetPos[1]*percent
        model_state_pub.pose.position.z = Curr_Pos[2]*(1-percent) + targetPos[2]*percent
        pub.publish(model_state_pub)
        time.sleep(0.001)


def run():
    rate = rospy.Rate(10000)
    def_frame = "World"
    hangup  = True
    base_dir = rospy.get_param('~data_position')
    res = np.loadtxt(base_dir)


    if def_frame == "World":
        model_state_pub.pose.position.x = 0.0
        model_state_pub.pose.position.y = 0.0
        model_state_pub.pose.position.z = 0.5

        model_state_pub.pose.orientation.x = 0.0
        model_state_pub.pose.orientation.y = 0.0
        model_state_pub.pose.orientation.z = 0.0
        model_state_pub.pose.orientation.w = 1.0

        model_state_pub.reference_frame = "world"

        time_ms = 0
        period = 5000
        radius = 1.5

        # while not rospy.is_shutdown():
        #     model_state_pub.pose.position.x = radius * math.sin(2*math.pi * time_ms/period)
        #     model_state_pub.pose.position.y = radius * math.cos(2*math.pi * time_ms/period)
        #     quaternion = quaternion_from_euler(0,0,-2*math.pi*time_ms/period)
        #     model_state_pub.pose.orientation.x = quaternion[0]
        #     model_state_pub.pose.orientation.y = quaternion[1]
        #     model_state_pub.pose.orientation.z = quaternion[2]
        #     model_state_pub.pose.orientation.w = quaternion[3]

        #     pub.publish(model_state_pub)
        #     rate.sleep()
        #     time_ms +=1
        # if hangup:
        # while not rospy.is_shutdown():
        #     pub.publish(model_state_pub)
        #     rate.sleep()
        # else:
        for i in range(1,res.shape[0]//48):
            curr_Pos = [res[48*(i-1)+6],res[48*(i-1)+7],res[48*(i-1)+8]+0.2]
            target_Pos = [res[48*i+6],res[48*i+7],res[48*i+8]+0.2]
            moveBasePosition(curr_Pos, target_Pos, 100)


            
    elif def_frame == "Robot":
        model_state_pub.twist.linear.x = 0.0
        model_state_pub.twist.linear.y = 0.0
        model_state_pub.twist.linear.z = 0.1

        model_state_pub.twist.angular.x = 0.0
        model_state_pub.twist.angular.y = 0.0
        model_state_pub.twist.angular.z = 0.0

        model_state_pub.reference_frame = "base"

        while not rospy.is_shutdown():
            pub.publish(model_state_pub)
            rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass