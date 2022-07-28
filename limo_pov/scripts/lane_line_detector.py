#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math

#Equation to find the gradient of a line from two points
def gradient(pt1, pt2):
    return float(pt2[1] - pt1[1]) / (pt2[0] - pt1[0])

def lane():
    #load the image from the disk
    img = cv2.imread("/home/syamim/catkin_ws/src/limo_ros/limo_pov/assets/road_lane.jpg")
    #make a copy for display with lines
    lineImg = np.copy(img)
    #remove color by converting to grayscale
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #filter low intensity whites
    img = cv2.inRange(img, 250, 255)
    #apply gaussian blur to smoothen lines
    cv2.GaussianBlur(img, (15, 15), 0)
    #apply canny filtering to detect edges
    lower_hysteresis = 100
    upper_hysteresis = 150
    edges = cv2.Canny(img, lower_hysteresis, upper_hysteresis)
    #apply HoughLinesP to directly obtain line end points
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=150, minLineLength=200, maxLineGap=30)
    
    lAng = 0
    rAng = 0
    lAngN = 0
    rAngN = 0
    lx2 = 0
    ly2 = 0
    rx2 = 0
    ry2 = 0

    for points in lines:
      # Extracted points nested in the list
        x1,y1,x2,y2=points[0]
        # Draw the lines joing the points
        # On the original image
        cv2.line(lineImg,(x1,y1),(x2,y2),(0,0,255),3, cv2.LINE_AA)
        ang = gradient((x1, y1), (x2, y2))
        if ang < 0:
            lAng = lAng + ang
            lAngN = lAngN + 1
            lx2 = lx2 + x2
            ly2 = ly2 + y2
        else:
            rAng = rAng + ang
            rAngN = rAngN + 1
            rx2 = rx2 + x1
            ry2 = ry2 + y1

    #find the line averages
    lAngSum = str(lAng / lAngN)
    rAngSum = str(rAng / rAngN)
    lx2 = lx2 / int(lAngN * 0.9)
    ly2 = ly2 / lAngN
    rx2 = rx2 / int(rAngN * 1.25)
    ry2 = ry2 / rAngN
    #coords for the text
    lorg = (lx2, ly2)
    rorg = (rx2, ry2)
    #draw the text
    cv2.putText(lineImg, lAngSum, lorg, cv2.FONT_HERSHEY_PLAIN, 1, [0, 0, 255], 2, cv2.LINE_AA)
    cv2.putText(lineImg, rAngSum, rorg, cv2.FONT_HERSHEY_PLAIN, 1, [0, 0, 255], 2, cv2.LINE_AA)
    #print the image with the lines
    cv2.imshow("Road", lineImg)

    cv2.waitKey(0)

if __name__ == '__main__':
    lane()