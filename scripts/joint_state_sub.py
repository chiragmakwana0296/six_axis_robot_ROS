#!/usr/bin/env python
#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2018 Chiragkumar Makwana.
# Released under the BSD License.
#
# Author:
#  Chiragkumar Makwana

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import serial
import math
import struct
import matplotlib.pyplot as plt

#ArduinoData = serial.Serial('/dev/ttyACM0',115200) #Create Serial port object called arduinoSerialData
ang_rad=[]
PI = math.pi
def callback_joint(data):
    ang0 = data.position[0]
    ang1 = data.position[1]
    ang2 = data.position[2]
    ang3 = data.position[3]
    ang4 = data.position[4]
    ang5 = data.position[5]
    ang6 = data.position[6]
    ang7 = data.position[7]
    ang8 = data.position[8]
    ang9 = data.position[9]


    #time.sleep(10)
    #rospy.loginfo('ang0: %f , ang1: %f''ang2: %f , ang3: %f''ang4: %f , ang5: %f', ang0,ang1,ang2,ang3,ang4,ang5
    joints_deg = [ang0, ang1, ang2, ang3, ang4, ang5, ang6, ang7, ang8, ang9]
    rospy.loginfo(joints_deg)
    #rospy.loginfo( joints_rad)
    #ArduinoData.write(deg_0)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('JointState_subscriber', anonymous=False)
    rospy.Subscriber('/JointState', JointState, callback_joint)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
