#!/usr/bin/env python

import rospy
import cv2
import numpy as np

def mix():
    img = cv2.imread("/home/syamim/catkin_ws/src/limo_ros/limo_pov/assets/color_venn_diagram.jpg")
    hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow("Original", img)
    red_lower = np.array([0, 25, 25])
    red_upper = np.array([15, 255, 255])
    green_lower = np.array([60, 25, 25])
    green_upper = np.array([80, 255, 255])
    blue_lower = np.array([110, 25, 25])
    blue_upper = np.array([130, 255, 255])
    yellow_lower = np.array([20, 25, 25])
    yellow_upper = np.array([30, 255, 255])
    cyan_lower = np.array([90, 25, 25])
    cyan_upper = np.array([100, 255, 255])
    magenta_lower = np.array([150, 25, 25])
    magenta_upper = np.array([160, 255, 255])


    R = cv2.inRange(hsvimg, red_lower, red_upper)
    G = cv2.inRange(hsvimg, green_lower, green_upper)
    B = cv2.inRange(hsvimg, blue_lower, blue_upper)
    Y = cv2.inRange(hsvimg, yellow_lower, yellow_upper)
    C = cv2.inRange(hsvimg, cyan_lower, cyan_upper)
    M = cv2.inRange(hsvimg, magenta_lower, magenta_upper)

    red_mask = cv2.bitwise_and(hsvimg, hsvimg, mask = R)
    green_mask = cv2.bitwise_and(hsvimg, hsvimg, mask = G)
    blue_mask = cv2.bitwise_and(hsvimg, hsvimg, mask = B)
    yellow_mask = cv2.bitwise_and(hsvimg, hsvimg, mask = Y)
    cyan_mask = cv2.bitwise_and(hsvimg, hsvimg, mask = C)
    magenta_mask = cv2.bitwise_and(hsvimg, hsvimg, mask = M)

    red = cv2.cvtColor(red_mask, cv2.COLOR_HSV2BGR)
    green = cv2.cvtColor(green_mask, cv2.COLOR_HSV2BGR)
    blue = cv2.cvtColor(blue_mask, cv2.COLOR_HSV2BGR)
    yellow = cv2.cvtColor(yellow_mask, cv2.COLOR_HSV2BGR)
    cyan = cv2.cvtColor(cyan_mask, cv2.COLOR_HSV2BGR)
    magenta = cv2.cvtColor(magenta_mask, cv2.COLOR_HSV2BGR)

    cv2.imshow("Red", red)
    cv2.imshow("Green", green)
    cv2.imshow("Blue", blue)
    cv2.imshow("Yellow", yellow)
    cv2.imshow("Cyan", cyan)
    cv2.imshow("Magenta", magenta)

    cv2.waitKey(0)

if __name__ == '__main__':
    mix()