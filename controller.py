#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 5a of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[  HB_2359 ]
# Author List:		[ Sriram Thangavel R, Akshith S, Sree Harish R S, Prasannakumar N ]
# Filename:		controller.py

################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from cv_basics.msg import aruco_data
import socket
import cv2

import math

################## GLOBAL VARIABLES ######################

PI = 3.14

x_goals = [250, 350, 150, 150, 350]    
y_goals = [250, 300, 300, 150, 150]
theta_goals = [0, PI/4, 3*PI/4, -3*PI/4, -PI/4]


#positions and orientation
hola_x = 0
hola_y = 0
hola_theta = 0

#distance from the center to the wheel
d =  0.17483
r = 1


#wheel angle 
wheel_angle = PI/3

#kp ratio
l_kp = 10
r_kp = 250

#kp values
kp_x = l_kp*2
kp_y = l_kp*2
kp_z = r_kp*2

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
	global hola_x,hola_y,hola_theta
	hola_x = msg.x
	hola_y = msg.y
	hola_theta = round(msg.theta,2)



def inverse_kinematics(vel_x,vel_y,vel_z):
	vel_front_wheel = (-d*vel_z + vel_x)/r
	vel_right_wheel = (-d*vel_z - math.cos(wheel_angle)*vel_x - math.sin(wheel_angle)*vel_y)/r
	vel_left_wheel = (-d*vel_z - math.cos(wheel_angle)*vel_x + math.sin(wheel_angle)*vel_y)/r
	return vel_front_wheel, vel_right_wheel, vel_left_wheel



def main():

	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)


	rospy.Subscriber('/detected_aruco',aruco_data,aruco_feedback_Cb)

	
	rate = rospy.Rate(10)   

	index = 0
	with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.bind((ip, 8002))
		s.listen()
		connection, address = s.accept()
		with connection:
			rospy.loginfo(f"Connected by {address}")


			while not rospy.is_shutdown():
					
					if (not hola_x):
						connection.sendall(str.encode(f'{0} {0} {0}'))
						rospy.loginfo("failure in feedback.................")
                    
					if hola_x:	# Error in global frame
						error_g_x = x_goals[index] - hola_x
						error_g_y = y_goals[index] - hola_y
						error_g_theta = theta_goals[index] - hola_theta

						error_b_x = error_g_x*math.cos(hola_theta) + error_g_y*math.sin(hola_theta)
						error_b_y = -error_g_x*math.sin(hola_theta) + error_g_y*math.cos(hola_theta)
						error_b_theta = error_g_theta


						vel_x = kp_x * error_b_x
						vel_y = kp_y * error_b_y
						vel_z = kp_z * error_b_theta


						vel_front, vel_right, vel_left= inverse_kinematics(vel_x,vel_y,vel_z)
				
						connection.sendall(str.encode(f'{vel_front} {vel_right} {vel_left}'))


						data = connection.recv(1024)
						
						rospy.loginfo(data)
		
					# position and orientation conditions
					x_condition_p = x_goals[index]-3 <= hola_x <= x_goals[index]+3
					y_condition_p = y_goals[index]-3 <= hola_y <= y_goals[index]+3
					theta_condition_p = theta_goals[index] - 0.05 <= hola_theta <= theta_goals[index] + 0.05

					if x_condition_p:
						vel_x = 0
						vel_front, vel_right, vel_left= inverse_kinematics(0,vel_y,vel_z)

						connection.sendall(str.encode(f'{vel_front} {vel_right} {vel_left}'))

					
					if y_condition_p:
						vel_y = 0
						vel_front, vel_right, vel_left= inverse_kinematics(vel_x,0,vel_z)
						connection.sendall(str.encode(f'{vel_front} {vel_right} {vel_left}'))
						
					
					if theta_condition_p:
						vel_z = 0
						vel_front, vel_right, vel_left= inverse_kinematics(vel_x,vel_y,0)
						connection.sendall(str.encode(f'{vel_front} {vel_right} {vel_left}'))

						
						

					if x_condition_p and y_condition_p  and theta_condition_p:
						connection.sendall(str.encode(f'{0} {0} {0}'))
						rospy.loginfo("Goal reached !!!!")
						rospy.sleep(1)

						if index < len(x_goals)-1:
							index += 1

					rate.sleep()

    ############################################


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

