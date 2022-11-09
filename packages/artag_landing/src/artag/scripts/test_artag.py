#!/usr/bin/env python3
import rospy
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import cv2.aruco as aruco

bridge = CvBridge()

DIM=(848, 800)
K=np.array([[293.56605526404223, 0.0, 416.467050053862], [0.0, 292.28573776006357, 390.77858553508827], [0.0, 0.0, 1.0]])
D=np.array([[0.03874407649229009], [-0.02923299780386297], [0.00721280327896209], [-0.00530800331302922]])

def findArucoMarkers(img, markerSize = 6, totalMarkers=250, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    # print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    return [bboxs, ids]

def ArucoYPR(img, aruco_bbox):
    if np.all(aruco_bbox[1] is not None):  # If there are markers found by detector
        for i in range(0, len(aruco_bbox[1])):  # Iterate in markers
            camera_matrix = [[609.6528165, 0, 339.75718769],
                            [0, 614.07063148, 148.0975697 ],
                            [0, 0, 1]]
            dist_coeffs = [[-0.05988835, 1.52152529, -0.01259443, 0.00812758, -5.56246644]]
            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(aruco_bbox[0][i], 0.02, np.array(camera_matrix), np.array(dist_coeffs))
            (rvec - tvec).any()  # get rid of that nasty numpy value array error
            aruco.drawDetectedMarkers(img, aruco_bbox[0])  # Draw A square around the markers
            aruco.drawAxis(img, np.array(camera_matrix), np.array(dist_coeffs), rvec, tvec, 0.01)  # Draw Axis

            rotM = np.zeros(shape=(3,3))
            cv2.Rodrigues(rvec[i-1], rotM, jacobian = 0)

            ypr = cv2.RQDecomp3x3(rotM)
            yaw = ypr[0][0]
            pitch = ypr[0][1]
            roll = ypr[0][2]

            print("Aruco ID : \n")
            print(i)
            print("Aruco Yaw : \n")
            print(yaw)
            print("Aruco Pitch : \n")
            print(pitch)
            print("Aruco Roll : \n")
            print(roll)

def ArucoCenter(img, aruco_bbox):
    (h, w) = img.shape[:2]
    if np.all(aruco_bbox[1] is not None):  # If there are markers found by detector
        for i in range(0, len(aruco_bbox[1])):  # Iterate in markers
            x_sum = aruco_bbox[0][0][0][0]+ aruco_bbox[0][0][1][0]+ aruco_bbox[0][0][2][0]+ aruco_bbox[0][0][3][0]
            y_sum = aruco_bbox[0][0][0][1]+ aruco_bbox[0][0][1][1]+ aruco_bbox[0][0][2][1]+ aruco_bbox[0][0][3][1]
                
            x_centerPixel = x_sum*.25
            y_centerPixel = y_sum*.25
            cv2.circle(img, (int(w/2), int(h/2)), 10, (0, 255, 0), 2)
            cv2.circle(img, (int(x_centerPixel), int(y_centerPixel)), 5, (255, 255, 255), -1)


def callback(data):
    global K, D, DIM

    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    arucofound = findArucoMarkers(undistorted_img)
    
    ArucoYPR(undistorted_img, arucofound)
    ArucoCenter(undistorted_img, arucofound)

    cv2.imshow("Artag", undistorted_img)

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