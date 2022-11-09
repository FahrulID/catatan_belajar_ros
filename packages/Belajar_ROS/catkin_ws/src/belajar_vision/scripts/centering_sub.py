#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
 
def nothing(x):
    pass

def callback(data):
  # Get current positions of all trackbars
  hMin = cv2.getTrackbarPos('HMin', 'HSV')
  sMin = cv2.getTrackbarPos('SMin', 'HSV')
  vMin = cv2.getTrackbarPos('VMin', 'HSV')
  hMax = cv2.getTrackbarPos('HMax', 'HSV')
  sMax = cv2.getTrackbarPos('SMax', 'HSV')
  vMax = cv2.getTrackbarPos('VMax', 'HSV')
  
  # Set minimum and maximum HSV values to display 
  lower = np.array([hMin, sMin, vMin])
  upper = np.array([hMax, sMax, vMax])
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")

  cv2.namedWindow('Live Leak (ಠಿ ͟ʖ ͡ಠ)', cv2.WINDOW_NORMAL)
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)

  # Convert to HSV format and color threshold
  hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(hsv, lower, upper)
  result = cv2.bitwise_and(current_frame, current_frame, mask=mask)
   
  # Display image
  cv2.imshow("Live Leak (ಠಿ ͟ʖ ͡ಠ)", result)
   
  cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('centering_sub', anonymous=True)

  cv2.namedWindow("HSV")
  cv2.resizeWindow("HSV", 400, 300)

  cv2.createTrackbar('HMin', 'HSV', 0, 360, nothing)
  cv2.createTrackbar('SMin', 'HSV', 0, 255, nothing)
  cv2.createTrackbar('VMin', 'HSV', 0, 255, nothing)
  cv2.createTrackbar('HMax', 'HSV', 360, 360, nothing)
  cv2.createTrackbar('SMax', 'HSV', 255, 255, nothing)
  cv2.createTrackbar('VMax', 'HSV', 255, 255, nothing)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
