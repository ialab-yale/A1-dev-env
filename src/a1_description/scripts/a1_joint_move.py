#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np 

import time
import tf2_ros
import geometry_msgs.msg 


joint_state = JointState()
pub = rospy.Publisher('joint_states',JointState, queue_size=10)

br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


def moveBasePosition(Curr_Pos_joint, targetPos_joint, Curr_Pos_base, targetPos_base, duration):
    for i in range(duration):
        if rospy.is_shutdown():
            break
        percent = i/duration
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 
                            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']
        position = Curr_Pos_joint*(1-percent) + targetPos_joint*percent
        # joint_state.position = [0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8]# position.tolist()
        joint_state.position = position.tolist()
        joint_state.velocity = []
        joint_state.effort = []


        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base"
        t.child_frame_id = "trunk"
        position_base = Curr_Pos_base*(1-percent) + targetPos_base*percent

        rot_base = get_quaternion_from_euler(position_base[3], position_base[4], position_base[5])
        # position_base = position_base.tolist()
        t.transform.translation.x = position_base[0] 
        t.transform.translation.y = position_base[1]
        t.transform.translation.z = position_base[2]
        t.transform.rotation.x = rot_base[0]
        t.transform.rotation.y = rot_base[1]
        t.transform.rotation.z = rot_base[2]
        t.transform.rotation.w = rot_base[3]
        br.sendTransform(t)
        pub.publish(joint_state)

        time.sleep(0.001)

def talker():
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(300)
    
    base_dir = rospy.get_param('~data_position')
    res = np.loadtxt(base_dir)

    # while not rospy.is_shutdown():
    #     joint_state.header = Header()
    #     joint_state.header.stamp = rospy.Time.now()
    #     joint_state.name = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 
    #                         'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    #                         'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    #                         'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'] #calf is lower and thigh is upper 
    #     joint_state.position=[-0.27, 2.09, -1.02, 
    #                         0.75, 2.27, -2.00, 
    #                         -0.75, 1.42, -1.64,
    #                         -0.57, -0.65, -1.97]
    #     joint_state.velocity = []
    #     joint_state.effort = []
    #     pub.publish(joint_state)
    #     rate.sleep()
    for j in range(100):
        for i in range(1,res.shape[0]//48):
            # curr_Pos = np.array([res[48*(i-1)+6],res[48*(i-1)+7],res[48*(i-1)+8]])
            curr_Pos_joint = res[48*(i-1)+12:48*(i-1)+24]
            target_Pos_joint = res[48*i+12:48*i+24]
            # target_Pos = np.array([res[48*i+6],res[48*i+7],res[48*i+8]])
            curr_Pos_base = res[48*(i-1)+6:48*(i-1)+12]
            target_Pos_base = res[48*i+6:48*i+12]
            moveBasePosition(curr_Pos_joint, target_Pos_joint, curr_Pos_base, target_Pos_base, 300)

        # moveBasePosition(target_Pos_joint, res[-12:], target_Pos_base, res[-18:-15], 300)

if __name__ =='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass