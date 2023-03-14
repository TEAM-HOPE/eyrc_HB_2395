#!/usr/bin/env python3


'''
* Team Id : HB_2359
* Author List : Sriram Thangavel, Sree Harish, Akshith, Prasannakumar
* Filename: Controller.py
* Theme: <Theme name -- Specific to eYRC / eYRCPlus >
* Functions: <Comma separated list of Functions defined in this file>
* Global Variables: <List of global variables defined in this file, none if no global
* variables>
'''


################### IMPORT MODULES #######################

import rospy
import sys
import cv2
from std_msgs.msg import String, Int32
from cv_basics.msg import aruco_data
import socket
import signal
import math
import numpy as np
import time

################### VARIABLES AND CONSTANTS #######################
PI = 3.14

xList, yList, xListFinal, yListFinal = [], [], [], []
theta_goals = [0]
pen_status = 0

# positions and orientation
hola_x = 0
hola_y = 0
hola_theta = 0
task_status = 0

penup = 0
taskstart = 0


# distance from the center to the wheel
d = 0.17483


# wheel angle
wheel_angle = PI/3

# kp ratio
l_kp = 27
r_kp = 250

# kp values
kp_x = l_kp*1
kp_y = l_kp*1
kp_z = r_kp*1

kd_x = 42
kd_y = 42
kd_z = 42

ki_x = 0
ki_y = 0
ki_z = 0

prev_error_x = 0
prev_error_y = 0
prev_error_z = 0

ip = '192.168.43.50'

##################### FUNCTION DEFINITIONS #######################


def signal_handler(sig, frame):
    rospy.loginfo('Clean-up !')
    cleanup()
    sys.exit(0)


def cleanup():
    socket.close()
    rospy.loginfo("cleanup done")


def aruco_feedback_Cb(msg):
    global hola_x, hola_y, hola_theta
    hola_x = msg.x
    hola_y = msg.y
    hola_theta = round(msg.theta, 2)


def thresh_img(img_path):
    img = cv2.imread(img_path)
    img = cv2.resize(img, (400, 400), interpolation=cv2.INTER_AREA)
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_img, 100, 200)
    ret, thresh = cv2.threshold(edges, 127, 255, 0)
    return ret, thresh


def get_contour(thresh):
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_ext, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours, contours_ext


def robot_contours():
    global xList, yList, xListFinal, yListFinal
    ret, thresh = thresh_img('/home/prasannakumar/Desktop/taskdrawing_ws/src/cv_basics/scripts/images/robotFinal.png')
    cnts, cnts_ext = get_contour(thresh)

    for i in cnts_ext:
        for j in i:
            xList.append(j[0][0])
            yList.append(j[0][1])
    xListFinal.append(xList)
    yListFinal.append(yList)

def pen_coordinate(xList):
     initial_coordinate = xList[0]
     final_coordinate = xList[-1]
     return initial_coordinate,final_coordinate

def pen(xList, index):

    global penup
    # initial_coordinate, final_coordinate = pen_coordinate(xList)
    if 0<index<len(xList)-1:
        penup = 1
    elif index == len(xList)-1:
        penup = 0
    return penup

def task_proc(xList, index):
    global taskstart
    initial_coordinate, final_coordinate = pen_coordinate(xList)
    if 0<index<len(xList)-1:
        taskstart = 0
    elif index == len(xList)-1:
        taskstart = 1
    return taskstart

def inverse_kinematics(vel_x, vel_y, vel_z):
    vel_front_wheel = (-d*vel_z + vel_x)
    vel_right_wheel = (-d*vel_z - math.cos(wheel_angle) *vel_x - math.sin(wheel_angle)*vel_y)
    vel_left_wheel = (-d*vel_z - math.cos(wheel_angle) *vel_x + math.sin(wheel_angle)*vel_y)
    return vel_front_wheel, vel_right_wheel, vel_left_wheel


def main():
    global xList, yList, xListFinal, yListFinal, task_status, pen_status
    integrated_error_x = 0
    integrated_error_y = 0
    integrated_error_z = 0

    rospy.init_node('controller_node')

    signal.signal(signal.SIGINT, signal_handler)

    rospy.Subscriber('/detected_aruco', aruco_data, aruco_feedback_Cb)

    contourPub = rospy.Publisher('/contours', String, queue_size=10)
    cData = String()

    penPub = rospy.Publisher('/penStatus', Int32, queue_size=10)
    penData = Int32()

    taskStatusPub = rospy.Publisher('/taskStatus', Int32, queue_size=10)
    taskStatus = Int32()

    rate = rospy.Rate(10)

    robot_contours()

    


    index = 0
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((ip, 8002))
        s.listen()
        connection, address = s.accept()
        with connection:
            rospy.loginfo(f"Connected by {address}")
            prev_time = time.time()

            while not rospy.is_shutdown():

                if (not hola_x):
                    connection.sendall(str.encode(f'{0} {0} {0} {pen_status}'))
                    rospy.loginfo("failure in feedback.................")

                if hola_x:  # Error in global frame

                    rospy.loginfo(50 + xList[index])
                    rospy.loginfo(50 + yList[index])

                    error_g_x = 50 + xList[index] - hola_x
                    error_g_y = 50 + yList[index] - hola_y
                    error_g_theta = theta_goals[0] - hola_theta

                    error_b_x = error_g_x * math.cos(hola_theta) + error_g_y*math.sin(hola_theta)
                    error_b_y = -error_g_x * math.sin(hola_theta) + error_g_y*math.cos(hola_theta)
                    error_b_theta = error_g_theta

                    # Calculate time difference
                    current_time = time.time()
                    dt = current_time - prev_time
                    prev_time = current_time

                    if index == 0:
                        delta_error_x = 0
                        delta_error_y = 0
                        delta_error_z = 0

                    else:
                        delta_error_x = (error_b_x - prev_error_x) / dt
                        delta_error_y = (error_b_y - prev_error_y) / dt
                        delta_error_z = (error_b_theta - prev_error_z) / dt

                    integrated_error_x += error_b_x * dt
                    integrated_error_y += error_b_y * dt
                    integrated_error_z += error_b_theta * dt

                    # Update previous errors
                    prev_error_x = error_b_x
                    prev_error_y = error_b_y
                    prev_error_z = error_b_theta

                    # Calculate control signals with PID controller
                    vel_x = (kp_x * error_b_x + kd_x * delta_error_x + ki_x * integrated_error_x)
                    vel_y = (kp_y * error_b_y + kd_y *delta_error_y + ki_y * integrated_error_y)
                    vel_z = (kp_z * error_b_theta + kd_z *delta_error_z + ki_z * integrated_error_z)

                    vel_front, vel_right, vel_left = inverse_kinematics(vel_x, vel_y, vel_z)

                    cData.data = str([xListFinal, yListFinal])
                    contourPub.publish(cData)

                    pen_status = pen(xList,index)
                    penData.data = pen_status
                    penPub.publish(penData)

                    task = task_proc(xList, index)
                    taskStatus.data = task
                    taskStatusPub.publish(taskStatus)

                    connection.sendall(str.encode(f'{vel_front} {vel_right} {vel_left} {pen_status}'))

                    data = connection.recv(1024)

                    x_condition_p =50 + xList[index]-4 <= hola_x <= 50 + xList[index]+4
                    y_condition_p =50 + yList[index]-4 <= hola_y <= 50 + yList[index]+4

                    if x_condition_p and y_condition_p:
                        rospy.loginfo("Goal reached !,current index: %d", index)

                        if index < len(xList)-1:
                            index += 1
                            rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass