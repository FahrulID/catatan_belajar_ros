#!/usr/bin/env python3
import rospy
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()

K = np.array([[292.61963059566585, 0.0, 434.06625328020726], [0.0, 293.47447585443257, 397.844887775816], [0.0, 0.0, 1.0]])
D = np.array([[0.05562957667341676], [-0.16881366101480275], [0.2079676007955449], [-0.07057623318631764]])
DIM = (848, 800)

def callback(data):
    global K, D, DIM

    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    cv2.imshow("Undistorted", undistorted_img)

    key = cv2.waitKey(10)

    if key==27: # 27 = Esc
        rospy.signal_shutdown("Done testing!")

def receive():
    rospy.Subscriber("/camera/fisheye1/image_raw", Image, callback)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("test" , anonymous=True , disable_signals=True)
    try:
        receive()
    except rospy.ROSInterruptException: pass
