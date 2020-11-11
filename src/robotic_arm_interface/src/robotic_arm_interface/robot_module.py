#!/usr/bin/env python
import rospy
import threading
import vrep
import time
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

class Robot():
    def __init__(self,connectionAddress='127.0.0.1' , connectionPort= 19997, waitUntilConnected=True, doNotReconnectOnceDisconnected=True, timeOutInMs=5000, commThreadCycleInMs=5):
                
        rospy.init_node('robot_arm_node')
        self.clientID = None
        self.target_sphere_handle = None
        self.axis_j1_handle = None
        self.axis_j2_handle = None
        self.axis_j3_handle = None
        self.axis_j4_handle = None
        self.axis_j5_handle = None
        self.axis_j6_handle = None
        self.target_handle = None
        self.tip_handle = None

        self.connectionAddress = connectionAddress 
        self.connectionPort = connectionPort
        self.waitUntilConnected = waitUntilConnected
        self.doNotReconnectOnceDisconnected = doNotReconnectOnceDisconnected
        self.timeOutInMs = timeOutInMs
        self.commThreadCycleInMs = commThreadCycleInMs

        t1 = threading.Thread(target=self.subscriber_thread)
        t1.start()
        self.joint_state_pub = rospy.Publisher("/JointStates",JointState,queue_size=1)

    def subscriber_thread(self):
        self.joystick_sub = rospy.Subscriber("/joy",Joy,self.joy_control_cb)
        rospy.spin()

    def joy_control_cb(self,data):
        returnCode,self.target_sphere_handle = vrep.simxGetObjectHandle(self.clientID,'Target_sphere',vrep.simx_opmode_oneshot_wait)
        position = []
        if returnCode == 0:
            returnCode,target_sphere_position=vrep.simxGetObjectPosition(self.clientID,self.target_sphere_handle,-1,vrep.simx_opmode_oneshot_wait)
            returnCode,target_sphere_orentation=vrep.simxGetObjectOrientation(self.clientID,self.target_sphere_handle,-1,vrep.simx_opmode_oneshot_wait)
            position[0] = target_sphere_position[0] +0.001*data.buttons[3] - 0.001*data.buttons[2]
            position[1] = target_sphere_position[1] +0.001*data.buttons[4] - 0.001*data.buttons[1]
            position[2] = target_sphere_position[2] +0.001*data.buttons[6] - 0.001*data.buttons[5]
            orentation[0] = target_sphere_orentation[0] +0.01*data.buttons[14] - 0.01*data.buttons[15]
            orentation[1] = target_sphere_orentation[1] +0.001*data.buttons[4] - 0.001*data.buttons[1]
            orentation[2] = target_sphere_orentation[2] +0.01*data.buttons[12] - 0.01*data.buttons[13]

        vrep.simxSetObjectPosition(self.clientID,self.target_sphere_handle,-1,position,vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectOrientation(self.clientID,self.target_sphere_handle,-1,orentation,vrep.simx_opmode_oneshot_wait)

    def get_object_handle(self):
        returnCode,self.axis_handle[0] = vrep.simxGetObjectHandle(self.clientID,'axis_j1',vrep.simx_opmode_oneshot_wait)
        returnCode,self.axis_handle[1] = vrep.simxGetObjectHandle(self.clientID,'axis_j2',vrep.simx_opmode_oneshot_wait)
        returnCode,self.axis_handle[2] = vrep.simxGetObjectHandle(self.clientID,'axis_j3',vrep.simx_opmode_oneshot_wait)
        returnCode,self.axis_handle[3] = vrep.simxGetObjectHandle(self.clientID,'axis_j4',vrep.simx_opmode_oneshot_wait)
        returnCode,self.axis_handle[4] = vrep.simxGetObjectHandle(self.clientID,'axis_j5',vrep.simx_opmode_oneshot_wait)
        returnCode,self.axis_handle[5] = vrep.simxGetObjectHandle(self.clientID,'axis_j6',vrep.simx_opmode_oneshot_wait)
        
        #returnCode,self.target_handle = vrep.simxGetObjectHandle(self.clientID,'Target',vrep.simx_opmode_oneshot_wait)
        #returnCode,self.tip_handle = vrep.simxGetObjectHandle(self.clientID,'Tip',vrep.simx_opmode_oneshot_wait)
        return axis_handle

    def pub_joint_states(self):
        joint_position = []
        Xmin = -2*math.pi
        Xmax = 2*math.pi
        Ymin =  -6400*25
        Ymax =  6400*25
        joints = self.get_object_handle()
        for joint in joints:
            joint_position = vrep.simxGetJointPosition(self.clientID, joint, vrep.simx_opmode_oneshot_wait)
            joint_position.append(map_joint_angle(joint_position,Xmin,Xmax,Ymin,Ymax))
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.time()
        joint_state_msg.header.frame_id = "Robot_frame"
        joint_state_msg.position = joint_position
        self.joint_state_pub.publish(joint_state_msg)

    def map_joint_angle(self,value,xmin,xmax,ymin,ymax):
        out= ( ((value-xmin)*(ymax-ymin))/(xmax-xmin) )+ymin
        return out

    def connect(self):
        vrep.simxFinish(-1) #close all opened connections

        self.clientID=vrep.simxStart(self.connectionAddress, self.connectionPort, self.waitUntilConnected, self.doNotReconnectOnceDisconnected, self.timeOutInMs, self.commThreadCycleInMs) # Connect to V-REP
        if self.clientID!=-1:
            print ('Connected to remote API server')
            while True:
                try:
                    pub_joint_states()
                except rospy.ROSInterruptException:
                    pass        
        else:
            rospy.loginfo("Failed connecting to remote API server")


"""
if __name__ == "__main__":
    R = Robot()
    R.connect()

"""


