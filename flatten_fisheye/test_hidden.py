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

balance = 0.0
dim2 = None
dim3 = None

def callback(data):
    global K, D, DIM, balance, dim2, dim3

    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    # #
    # NK = K.copy()
    # NK[0,0]=K[0,0]/2
    # NK[1,1]=K[1,1]/2

    # dim1 = frame.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    # scaled_K = K * 1 / .45  # The values of K is to scale with image dimension.
    # scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # # Just by scaling the matrix coefficients!
    # new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, DIM, np.eye(3), balance=1)

    # map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, DIM, cv2.CV_16SC2)  # Pass k in 1st parameter, nk in 4th parameter
    # nemImg = cv2.remap( frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    # cv2.imshow("Undistorted", nemImg)



    img = frame
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)


    key = cv2.waitKey(0)
    #

    if key==27: # 27 = Esc
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Done testing!")

def receive():
    rospy.Subscriber("/camera/fisheye1/image_raw", Image, callback)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("test_hidden" , anonymous=True , disable_signals=True)
    try:
        receive()
    except rospy.ROSInterruptException: pass
