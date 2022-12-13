#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

from pickle import NONE
import cv2
import numpy as np
import RPi.GPIO as GPIO
import sys
from matplotlib import pyplot as plt
from numpy.random import *
import time
import rospy
#from package_test.msg import Adder
import rospy
from std_msgs.msg import Float64
from package_test.msg import Status
from geometry_msgs.msg import Pose2D
import statistics


duty = 85
node_cycle = 100
x_list = []
y_list = []
move = 0
robot_mode = 0
robot_status = Status()


#GPIO initial set
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)

#GPIO SWitch set
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)



p1 = GPIO.PWM(27, 10000) #50Hz
p2 = GPIO.PWM(22, 10000) #50Hz
p3 = GPIO.PWM(23, 10000) #50Hz
p4 = GPIO.PWM(24, 10000) #50Hz

p1.start(0)
p2.start(0)
p3.start(0)
p4.start(0)

def callback(data):
    global x_list ,y_list,robot_status
    robot_status.stuck1 = data.stuck_global
    #print(data.x)
    x_list.append(data.x)
    y_list.append(data.y)
    xn = x_list[-1]
    yn = y_list[-1]
    x100 = x_list[-100:]
    y100 = y_list[-100:]

    x_average = statistics.mean(x100)
    y_average = statistics.mean(y100)

    s_x = abs(xn - x_average)
    s_y =abs(yn - y_average)

    if s_x or s_y < 0.03 :
        move = 0
    else :
        move = 1

    if robot_mode == 1 :
        
        p1.ChangeDutyCycle(duty)
        p2.ChangeDutyCycle(0)
        p3.ChangeDutyCycle(duty)
        p4.ChangeDutyCycle(0)

        if move == 1 :
            robot_status.stuck1 = 0
        else :
            robot_status.stuck1 = 1

    if robot_mode == 0 :

        p1.ChangeDutyCycle(0)
        p2.ChangeDutyCycle(0)
        p3.ChangeDutyCycle(0)
        p4.ChangeDutyCycle(0)

        if move == 1 :
            robot_status.stuck1 = 1
        else :
            robot_status.stuck1 = 0




    time.sleep(0.01)


def status():
    rospy.init_node('status', anonymous=True)
    cycle_rate = rospy.Rate(node_cycle)

    # Subscriberとしてmocap_nodeというnode no Robot_1/ground_poseというトピックに対してSubscribeし、トピックが更新されたときは
    # callbackという名前のコールバック関数を実行する

    rospy.Subscriber('mocap_node/Robot_2/ground_pose', Pose2D, callback)
    pub = rospy.Publisher('robot_status', Status, queue_size = 3)



    while not rospy.is_shutdown():

        pub.publish(robot_status)
        

        cycle_rate.sleep()

    # トピック更新の待ちうけを行う関数
    #rospy.spin()

if __name__ == '__main__':
    try:
        status()

    except rospy.ROSInterruptException: pass
