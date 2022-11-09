#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

directory = ""
bridge = CvBridge()

i = 0

def callback(data):
    global i
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("win", frame)
    key = cv2.waitKey(10)

    if key==27: # 27 = Esc
        rospy.signal_shutdown("Done capturing for calibration!")
    elif key==32: # 32 = Space
        out_path = f"{directory}/Frame{i}.jpg"
        cv2.imwrite(out_path, frame)
        rospy.loginfo("Frame saved on %s", directory)
        i+=1

def receive():
    rospy.Subscriber("/camera/fisheye1/image_raw", Image, callback)
    rospy.spin()

def folder():
    if not os.path.exists(directory):
        os.makedirs(directory)

if __name__ == "__main__":
    rospy.init_node("capture" , anonymous=True , disable_signals=True)
    directory = rospy.get_param('~directory')
    try:
        folder()
        receive()
    except rospy.ROSInterruptException: pass