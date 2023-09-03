#!/usr/bin/env python

import rospy 
from unitree_legged_msgs.msg import LowCmd
from unitree_legged_msgs.msg import LowState
from unitree_legged_msgs.msg import MotorCmd
from unitree_legged_msgs.msg import MotorState
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import time
import numpy as np 

lowState = LowState()
lowCmd = LowCmd()

lowState_pub = rospy.Publisher("/a1_gazebo/lowState/state", LowState, queue_size=1)

servo_pub_FR_hip = rospy.Publisher("/a1_gazebo/FR_hip_controller/command", MotorCmd ,queue_size=1)
servo_pub_FR_thigh = rospy.Publisher("/a1_gazebo/FR_thigh_controller/command", MotorCmd ,queue_size=1)
servo_pub_FR_calf = rospy.Publisher("/a1_gazebo/FR_calf_controller/command", MotorCmd ,queue_size=1)
servo_pub_FL_hip = rospy.Publisher("/a1_gazebo/FL_hip_controller/command", MotorCmd ,queue_size=1)
servo_pub_FL_thigh = rospy.Publisher("/a1_gazebo/FL_thigh_controller/command", MotorCmd ,queue_size=1)
servo_pub_FL_calf = rospy.Publisher("/a1_gazebo/FL_calf_controller/command", MotorCmd ,queue_size=1)
servo_pub_RR_hip = rospy.Publisher("/a1_gazebo/RR_hip_controller/command", MotorCmd ,queue_size=1)
servo_pub_RR_thigh = rospy.Publisher("/a1_gazebo/RR_thigh_controller/command", MotorCmd ,queue_size=1)
servo_pub_RR_calf = rospy.Publisher("/a1_gazebo/RR_calf_controller/command", MotorCmd ,queue_size=1)
servo_pub_RL_hip = rospy.Publisher("/a1_gazebo/RL_hip_controller/command", MotorCmd ,queue_size=1)
servo_pub_RL_thigh = rospy.Publisher("/a1_gazebo/RL_thigh_controller/command", MotorCmd ,queue_size=1)
servo_pub_RL_calf = rospy.Publisher("/a1_gazebo/RL_calf_controller/command", MotorCmd ,queue_size=1)

def paramInit():
    for i in range(4):
        lowCmd.motorCmd[i*3+0].mode = 10
        lowCmd.motorCmd[i*3+0].Kp = 70
        lowCmd.motorCmd[i*3+0].dq = 0
        lowCmd.motorCmd[i*3+0].Kd = 3
        lowCmd.motorCmd[i*3+0].tau = 0
        lowCmd.motorCmd[i*3+1].mode = 10
        lowCmd.motorCmd[i*3+1].Kp = 180
        lowCmd.motorCmd[i*3+1].dq = 0
        lowCmd.motorCmd[i*3+1].Kd = 8
        lowCmd.motorCmd[i*3+1].tau = 0
        lowCmd.motorCmd[i*3+2].mode = 10
        lowCmd.motorCmd[i*3+2].Kp = 300
        lowCmd.motorCmd[i*3+2].dq = 0
        lowCmd.motorCmd[i*3+2].Kd = 15
        lowCmd.motorCmd[i*3+2].tau = 0
    for i in range(12):
        lowCmd.motorCmd[i].q = lowState.motorState[i].q

def stand():
    pos = [0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
            0.0, 0.67, -1.3, -0.0, 0.67, -1.3]
    moveAllPosition(pos, 2000)

def moveAllPosition(targetPos, duration):
    lastPos = [lowState.motorState[j].q for j in range(12)]
    for i in range(duration):
        if rospy.is_shutdown():
            break
        percent = i/duration
        for j in range(12):
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent
        sendServoCmd()
        time.sleep(0.001)

def sendServoCmd():
    servo_pub_FR_hip.publish(lowCmd.motorCmd[0])
    servo_pub_FR_thigh.publish(lowCmd.motorCmd[1])
    servo_pub_FR_calf.publish(lowCmd.motorCmd[2])
    servo_pub_FL_hip.publish(lowCmd.motorCmd[3])
    servo_pub_FL_thigh.publish(lowCmd.motorCmd[4])
    servo_pub_FL_calf.publish(lowCmd.motorCmd[5])
    servo_pub_RR_hip.publish(lowCmd.motorCmd[6])
    servo_pub_RR_thigh.publish(lowCmd.motorCmd[7])
    servo_pub_RR_calf.publish(lowCmd.motorCmd[8])
    servo_pub_RL_hip.publish(lowCmd.motorCmd[9])
    servo_pub_RL_thigh.publish(lowCmd.motorCmd[10])
    servo_pub_RL_calf.publish(lowCmd.motorCmd[11])
    # time.sleep()


def motion_init():
    paramInit()
    stand()

def imuCallback(data):
    lowState.imu.quaternion[0] = data.orientation.w
    lowState.imu.quaternion[1] = data.orientation.x
    lowState.imu.quaternion[2] = data.orientation.y
    lowState.imu.quaternion[3] = data.orientation.z

    lowState.imu.gyroscope[0] = data.angular_velocity.x
    lowState.imu.gyroscope[1] = data.angular_velocity.y
    lowState.imu.gyroscope[2] = data.angular_velocity.z
    
    lowState.imu.accelerometer[0] = data.linear_acceleration.x
    lowState.imu.accelerometer[1] = data.linear_acceleration.y
    lowState.imu.accelerometer[2] = data.linear_acceleration.z

def FRhipCallback(data):
    start_up = False
    lowState.motorState[0].mode = data.mode
    lowState.motorState[0].q = data.q
    lowState.motorState[0].dq = data.dq
    lowState.motorState[0].tauEst = data.tauEst

def FRthighCallback(data):
    lowState.motorState[1].mode = data.mode
    lowState.motorState[1].q = data.q
    lowState.motorState[1].dq = data.dq
    lowState.motorState[1].tauEst = data.tauEst

def FRcalfCallback(data):
    lowState.motorState[2].mode = data.mode
    lowState.motorState[2].q = data.q
    lowState.motorState[2].dq = data.dq
    lowState.motorState[2].tauEst = data.tauEst

def FLhipCallback(data):
    start_up = False
    lowState.motorState[3].mode = data.mode
    lowState.motorState[3].q = data.q
    lowState.motorState[3].dq = data.dq
    lowState.motorState[3].tauEst = data.tauEst

def FLthighCallback(data):
    lowState.motorState[4].mode = data.mode
    lowState.motorState[4].q = data.q
    lowState.motorState[4].dq = data.dq
    lowState.motorState[4].tauEst = data.tauEst

def FLcalfCallback(data):
    lowState.motorState[5].mode = data.mode
    lowState.motorState[5].q = data.q
    lowState.motorState[5].dq = data.dq
    lowState.motorState[5].tauEst = data.tauEst

def RRhipCallback(data):
    start_up = False
    lowState.motorState[6].mode = data.mode
    lowState.motorState[6].q = data.q
    lowState.motorState[6].dq = data.dq
    lowState.motorState[6].tauEst = data.tauEst

def RRthighCallback(data):
    lowState.motorState[7].mode = data.mode
    lowState.motorState[7].q = data.q
    lowState.motorState[7].dq = data.dq
    lowState.motorState[7].tauEst = data.tauEst

def RRcalfCallback(data):
    lowState.motorState[8].mode = data.mode
    lowState.motorState[8].q = data.q
    lowState.motorState[8].dq = data.dq
    lowState.motorState[8].tauEst = data.tauEst
    

def RLhipCallback(data):
    start_up = False
    lowState.motorState[9].mode = data.mode
    lowState.motorState[9].q = data.q
    lowState.motorState[9].dq = data.dq
    lowState.motorState[9].tauEst = data.tauEst
    

def RLthighCallback(data):
    lowState.motorState[10].mode = data.mode
    lowState.motorState[10].q = data.q
    lowState.motorState[10].dq = data.dq
    lowState.motorState[10].tauEst = data.tauEst
    # rospy.loginfo(data.q)

def RLcalfCallback(data):
    lowState.motorState[11].mode = data.mode
    lowState.motorState[11].q = data.q
    lowState.motorState[11].dq = data.dq
    lowState.motorState[11].tauEst = data.tauEst

def FRfootCallback(data):
    lowState.eeForce[0].x = data.wrench.force.x
    lowState.eeForce[0].y = data.wrench.force.y
    lowState.eeForce[0].z = data.wrench.force.z
    lowState.footForce[0] = data.wrench.force.z

def FLfootCallback(data):
    lowState.eeForce[0].x = data.wrench.force.x
    lowState.eeForce[0].y = data.wrench.force.y
    lowState.eeForce[0].z = data.wrench.force.z
    lowState.footForce[0] = data.wrench.force.z

def RRfootCallback(data):
    lowState.eeForce[0].x = data.wrench.force.x
    lowState.eeForce[0].y = data.wrench.force.y
    lowState.eeForce[0].z = data.wrench.force.z
    lowState.footForce[0] = data.wrench.force.z

def RLfootCallback(data):
    lowState.eeForce[0].x = data.wrench.force.x
    lowState.eeForce[0].y = data.wrench.force.y
    lowState.eeForce[0].z = data.wrench.force.z
    lowState.footForce[0] = data.wrench.force.z

# class multiThread():
#     def __init__(self, name) -> None:
#         self.robot_name = name
def control_logic():
    while not rospy.is_shutdown():
        lowState_pub.publish(lowState)
        sendServoCmd()
        

def load_data():
    base_dir = rospy.get_param('~data_position')
    res = np.loadtxt(base_dir)
    return res

def joint_state_visual():
    paramInit()
    res = load_data()
    for i in range(1,res.shape[0]//48):
        target_Pos = [ res[48*i+j] for j in range(12,24)]
        moveAllPosition(target_Pos, 100)



if __name__ == "__main__":
    print("started")
    motion_init_required = False
    imu_sub = rospy.Subscriber("/trunk_imu", Imu, imuCallback, queue_size=1)
    # time.sleep(1)

    rospy.init_node('unitree_gazebo_servo')
    # listen_publish_obj = multiThread("a1")
    # time.sleep(1)
    rate = rospy.Rate(1000)
    servo_subcriber_FRthigh = rospy.Subscriber("/a1_gazebo/FR_hip_controller/state", MotorState, FRhipCallback, queue_size=1)
    servo_subcriber_FRthigh = rospy.Subscriber("/a1_gazebo/FR_thigh_controller/state", MotorState, FRthighCallback, queue_size=1)
    
    servo_subcriber_FRcalf = rospy.Subscriber("/a1_gazebo/FR_calf_controller/state", MotorState, FRcalfCallback, queue_size=1)
    servo_subcribers_FLhip = rospy.Subscriber("/a1_gazebo/FL_hip_controller/state", MotorState, FLhipCallback, queue_size=1)
    servo_subcriber_FLthigh = rospy.Subscriber("/a1_gazebo/FL_thigh_controller/state", MotorState, FLthighCallback, queue_size=1)
    servo_subcriber_FLcalf = rospy.Subscriber("/a1_gazebo/FL_calf_controller/state", MotorState, FLcalfCallback, queue_size=1)
    servo_subcriber_RRhip = rospy.Subscriber("/a1_gazebo/RR_hip_controller/state", MotorState, RRhipCallback, queue_size=1)
    servo_subcriber_RRthigh = rospy.Subscriber("/a1_gazebo/RR_thigh_controller/state", MotorState, RRthighCallback, queue_size=1)
    servo_subcriber_RRcalf = rospy.Subscriber("/a1_gazebo/RR_calf_controller/state", MotorState, RRcalfCallback, queue_size=1)
    servo_subcriber_RLhip = rospy.Subscriber("/a1_gazebo/RL_hip_controller/state", MotorState, RLhipCallback, queue_size=1)
    servo_subcriber_RLthigh = rospy.Subscriber("/a1_gazebo/RL_thigh_controller/state", MotorState, RLthighCallback, queue_size=1)
    servo_subcriber_RLcalf = rospy.Subscriber("/a1_gazebo/RL_calf_controller/state", MotorState, RLcalfCallback, queue_size=1)
    
    footForce_sub_FR = rospy.Subscriber("/visual/FR_foot_contact/the_force", WrenchStamped, FRfootCallback, queue_size=1)
    footForce_sub_FL = rospy.Subscriber("/visual/FL_foot_contact/the_force", WrenchStamped, FLfootCallback, queue_size=1)
    footForce_sub_RR = rospy.Subscriber("/visual/RR_foot_contact/the_force", WrenchStamped, RRfootCallback, queue_size=1)
    footForce_sub_RL = rospy.Subscriber("/visual/RL_foot_contact/the_force", WrenchStamped, RLfootCallback, queue_size=1)

    if motion_init_required:
        motion_init()
    else:
        joint_state_visual()
    control_logic()

    rospy.spin()
    
    
    
    

    




