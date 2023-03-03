#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 2 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ HB_2359 ]
# Author List:		[Sriram Thangavel R, Akshith S, Sree Harish R S, Prasannakumar N]
# Filename:		feedback.py
# Functions:
#			[ detect_aruco_marker, aruco_centroid, aruco_orientation, callback, main ]
# Nodes:		detected_aruco overhead_cam/image_raw


######################## IMPORT MODULES ##########################
			
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import math
import numpy as np			
from cv_basics.msg import aruco_data	# Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################


aruco_4_corners = np.array([])
aruco_8_corners = np.array([])
aruco_10_corners = np.array([])
aruco_12_corners = np.array([])

top_right_corner = np.array([])
top_left_corner = np.array([])
bottom_right_corner = np.array([])
bottom_left_corner = np.array([])

aruco_publisher = rospy.Publisher('/detected_aruco', aruco_data,queue_size=10)
aruco_msg = aruco_data()

##################### FUNCTION DEFINITIONS #######################

#To find all aruco markers
def find_arucos(img, markerSize=4, totalMarkers=50, draw=True):
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(cv2.aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = cv2.aruco.getPredefinedDictionary(key)
    arucoParam = cv2.aruco.DetectorParameters()
    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParam)
    return corners, ids, rejected
    # try:
    #     corners, ids, rejected = cv2.aruco.detectMarkers(img,arucoDict, parameters=arucoParam)
    # except cv2.error:
    #     return None, None, None
    # if draw:
    #     cv2.aruco.drawDetectedMarkers(img, corners, ids)
    # return corners, ids, rejected


#To get corners of a aruco
def get_aruco(ids,corners,aruco_id):
    if ids is not None and aruco_id in ids:
         index = np.where(ids == aruco_id)[0][0]
         aruco_corners= corners[index][0]
         return aruco_corners
    return np.array([])
    
#To get hola position with orientation
def hola_position_with_orientation(robot_aruco_corners):
        if robot_aruco_corners.any():
            M = cv2.moments(robot_aruco_corners)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            mid_x = (robot_aruco_corners[1][0]+ robot_aruco_corners[2][0])/2
            mid_y = ((500 - robot_aruco_corners[1][1])+ (500 - robot_aruco_corners[2][1]))/2
            del_x = mid_x - cx
            del_y = mid_y - cy
            theta = math.degrees(math.atan2(del_y,del_x))
            return cx,cy,round(theta,3)
        return 0, 0 ,0


def callback(data):
    global hola_x,hola_y,hola_theta,aruco_4_corners,aruco_8_corners,aruco_10_corners,aruco_12_corners
    global bottom_left_corner,bottom_right_corner,top_right_corner,top_left_corner
    br = CvBridge()
    rospy.loginfo("receiving camera frame")
    frame = br.imgmsg_to_cv2(data, "mono8")
    corners ,ids ,rej_img = find_arucos(frame)
   


    if (not aruco_4_corners.any()) or (not aruco_8_corners.any()) or (not aruco_10_corners.any()) or (not aruco_12_corners.any()):
        #store corners of each marker
        aruco_4_corners = get_aruco(ids,corners,4)
        aruco_8_corners = get_aruco(ids,corners,8)
        aruco_10_corners = get_aruco(ids,corners,10)
        aruco_12_corners = get_aruco(ids,corners,12)
        rospy.loginfo("hi")

        if aruco_4_corners.any() and aruco_8_corners.any() and aruco_10_corners.any()and aruco_12_corners.any():
        #corner of area
            top_left_corner = aruco_4_corners[0]
            top_right_corner = aruco_8_corners[0]
            bottom_right_corner = aruco_10_corners[0]
            bottom_left_corner = aruco_12_corners[0]

    if top_right_corner.any() and bottom_left_corner.any() and bottom_right_corner.any() and top_left_corner.any():
                #perspective transform parameters
                inital_arena = np.float32([top_left_corner,top_right_corner,bottom_left_corner,bottom_right_corner])
                expected_arena = np.float32([[0,0],[500,0],[0,500],[500,500]])
                M = cv2.getPerspectiveTransform(inital_arena,expected_arena)
                frame = cv2.warpPerspective(frame,M,(500,500))
                cv2.circle(frame,(250,250),10,(0,0,255),-1) #to draw dot at 250,250

    cv2.imshow("frame",frame)
    cv2.waitKey(1)
    new_corners ,new_ids ,new_rej_img = find_arucos(frame)
    aruco_hola_corners = get_aruco(new_ids,new_corners,15)
    hola_x , hola_y ,hola_theta = hola_position_with_orientation(aruco_hola_corners)
    
    aruco_msg.x = hola_x
    aruco_msg.y = hola_y
    aruco_msg.theta =  hola_theta
    aruco_publisher.publish(aruco_msg)


def main():
     rospy.init_node('aruco_feedback_node')
     rospy.Subscriber('/usb_cam/image_rect',Image,callback)
     rospy.spin()

if __name__ == '__main__':
     main()
