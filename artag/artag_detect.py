import cv2
import cv2.aruco as aruco
import numpy as np
import os
import math

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

# not needed for a moment. i wanna use this to do centering between two obstacles, but yeah it's not really necessary
def dualArucoCentering(img, aruco_bbox):
    l1 = aruco_bbox[0][0][0][1][0] - aruco_bbox[0][0][0][0][0]
    w1 = aruco_bbox[0][0][0][2][1] - aruco_bbox[0][0][0][1][1]
    center1 = (int(aruco_bbox[0][0][0][0][0] + (l1//2)), int(aruco_bbox[0][0][0][0][1] + (w1//2)))
    l2 = aruco_bbox[0][1][0][1][0] - aruco_bbox[0][1][0][0][0]
    w2 = aruco_bbox[0][1][0][2][1] - aruco_bbox[0][1][0][1][1]
    center2 = (int(aruco_bbox[0][1][0][0][0] + (l2//2)), int(aruco_bbox[0][1][0][0][1] + (w2//2)))
    center3 = ( center1[0] + int((center2[0] - center1[0]) / 2), center1[1] + int((center2[1] - center1[1]) / 2))

    cv2.circle(img, center1, 1, (255, 255, 0), 3)
    cv2.circle(img, center2, 1, (255, 255, 0), 3)
    cv2.circle(img, center3, 1, (0, 0, 255), 10)
    cv2.line(img, center1, center2, (255, 255, 0), 3)

    
def arucoCentering(img, aruco_bbox, id = 1):
    if id == 1:
        x3 = aruco_bbox[0][0][0][3][0]
        x2 = aruco_bbox[0][0][0][2][0]
        x1 = aruco_bbox[0][0][0][1][0]
        x0 = aruco_bbox[0][0][0][0][0]
        y3 = aruco_bbox[0][0][0][3][1]
        y2 = aruco_bbox[0][0][0][2][1]
        y1 = aruco_bbox[0][0][0][1][1]
        y0 = aruco_bbox[0][0][0][0][1]

        center_x = (x0+x1+x2+x3)/4
        center_y = (y0+y1+y2+y3)/4

        radius = math.sqrt((x0-center_x) ** 2 + (y0-center_y) ** 2)

        zero_x = center_x + (math.sqrt(2) / 2 * radius)
        zero_y = center_y - (math.sqrt(2) / 2 * radius)

        a2 = (center_x-zero_x) ** 2 + (center_y-zero_y) ** 2
        a = math.sqrt(a2)
        b2 = (zero_x-x0) ** 2 + (zero_y-y0) ** 2
        b = math.sqrt(b2)
        c2 = (x0-center_x) ** 2 + (y0-center_y) ** 2
        c = math.sqrt(c2)

        # alpha = math.acos((b2+c2-a2) / (2 * b * c))
        beta = math.acos((a2+c2-b2) / (2 * a * c))
        # gamma = math.acos((a2+b2-c2) / (2*a*b))

        # alpha = alpha * 180 / math.pi
        beta = beta * 180 / math.pi
        # gamma = gamma * 180 / math.pi

        if(x1 > x0): # Clockwise : x1 > x0, counter clockwise : x0 > x1
            beta = 360 - beta


        cv2.circle(img, (int(center_x), int(center_y)), 1, (0, 255, 255), 10)
        cv2.circle(img, (int(center_x), int(center_y)), int(radius), (0, 255, 255), 10)
        cv2.line(img, (int(center_x), int(center_y)), (int(zero_x), int(zero_y)), (0, 255, 255), 10)
        cv2.line(img, (int(center_x), int(center_y)), (int(x0), int(y0)), (0, 255, 255), 10)

        cv2.putText(img, str(int(beta)), (int(center_x), int(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 10, cv2.LINE_AA)
        # print("angle {0} {1} {2}".format(alpha, beta, gamma))
        return beta
    return NULL

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    while True:
        success, img = cap.read()
        findArucoMarkers(img)

        arucofound = findArucoMarkers(img)
        if len(arucofound[0])==1:
            angle = arucoCentering(img, arucofound)
            print(angle)
            
        cv2.imshow('img',img)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()