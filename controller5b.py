#!/usr/bin/env python3



import rospy
import sys
import cv2
from std_msgs.msg import String,Int32
from cv_basics.msg import aruco_data
import socket
import signal	
import math

PI = 3.14


xList , yList , xListFinal , yListFinal = [] , [] , [] , []
theta_goals = 0

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
	

def snapchat(img_path):
    img = cv2.imread(img_path,cv2.IMREAD_UNCHANGED)
    image = cv2.resize(img, (500,500), interpolation = cv2.INTER_AREA)
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_img, 100, 200)
    ret, thresh = cv2.threshold(edges, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def pen_coordinate(xList):
     list_length = len(xList)
     initial_coordinate = xList[0]
     final_coordinate = xList[list_length-1]
     return initial_coordinate,final_coordinate


def way_points(cnts):
        for i in cnts:  #cnts are the retrieved contours from cv2.findContours 
               xList.clear()    #clearing any previous data from the lists
               yList.clear()
    
               #iterating through the nested list
               for j in i:
        	#temporary appending the x,y coordinates in separate lists
                   xList.append(int(j[0][0]))
                   yList.append(int(j[0][1]))
    
               for x in xList:
                    xListFinal.append(x)
               
               for y in yList:
                    yListFinal.append(y)


def inverse_kinematics(vel_x,vel_y,vel_z):
	vel_front_wheel = (-d*vel_z + vel_x)/r
	vel_right_wheel = (-d*vel_z - math.cos(wheel_angle)*vel_x - math.sin(wheel_angle)*vel_y)/r
	vel_left_wheel = (-d*vel_z - math.cos(wheel_angle)*vel_x + math.sin(wheel_angle)*vel_y)/r
	return vel_front_wheel, vel_right_wheel, vel_left_wheel
    
      


def main():
    
    rospy.init_node('controller_node')

    signal.signal(signal.SIGINT, signal_handler)
    
    rospy.Subscriber('/detected_aruco',aruco_data,aruco_feedback_Cb)

    contourPub = rospy.Publisher('/contours',String,queue_size=10)
    cData = String()

    # penPub = rospy.Publisher('/penStatus', Int32, queue_size=10)
    # penData = Int32()
    
    # taskStatusPub = rospy.Publisher('/taskStatus', Int32, queue_size=10)
    # taskStatus = Int32()
    
    # rospy.Subscriber('endSignal',Int32,endSignalCb) #optional
    # taskStatus.data = 0 

    rate = rospy.Rate(10)  



    # Check if command line arguments are provided
    if len(sys.argv) == 3:

        if sys.argv[1] == 'img' and sys.argv[2] == '0':
              cnts = snapchat('/home/prasannakumar/Desktop/taskdrawing_ws/src/cv_basics/scripts/images/snapchat.png')
              way_points(cnts)
              cData.data = str([xListFinal,yListFinal])
              contourPub.publish(cData)

        if sys.argv[1] == 'img' and sys.argv[2] == '1':
              pass
              #smile

        if sys.argv[1] == 'fun' and sys.argv[2] == '0':
             pass
             #function mode

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
                        error_g_x = xList[index] - hola_x
                        error_g_y = yList[index] - hola_y
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

                        x_condition_p = xList[index]-3 <= hola_x <= xList[index]+3
                        y_condition_p = yList[index]-3 <= hola_y <= yList[index]+3
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
						

                            if index < len(xList)-1:
                                index += 1
                                rate.sleep()


        
       
       

  






if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

