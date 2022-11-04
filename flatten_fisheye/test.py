#!/usr/bin/env python3
import rospy
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()

DIM=(848, 800)
K=np.array([[293.56605526404223, 0.0, 416.467050053862], [0.0, 292.28573776006357, 390.77858553508827], [0.0, 0.0, 1.0]])
D=np.array([[0.03874407649229009], [-0.02923299780386297], [0.00721280327896209], [-0.00530800331302922]])

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
