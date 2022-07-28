#!/usr/bin/env python

import rospy
import cv2
import numpy as np

def pure_color_extractor():
    img = cv2.imread("/home/syamim/catkin_ws/src/limo_ros/limo_pov/assets/color_venn_diagram.jpg")

    cv2.imshow("image", img)

    B, G, R = cv2.split(img)

    red_img     = np.zeros(img.shape)
    blue_img    = np.zeros(img.shape)
    green_img   = np.zeros(img.shape)
    yellow_img  = np.zeros(img.shape)
    cyan_img    = np.zeros(img.shape)
    magenta_img = np.zeros(img.shape)

    yellow_img[:, :, 2] = cv2.bitwise_and(R, G)
    yellow_img[:, :, 1] = cv2.bitwise_and(R, G)

    cyan_img[:, :, 0] = cv2.bitwise_and(B, G)
    cyan_img[:, :, 1] = cv2.bitwise_and(B, G)

    magenta_img[:, :, 2] = cv2.bitwise_and(R, B)
    magenta_img[:, :, 0] = cv2.bitwise_and(R, B)

    red_img[:, :, 2] = R - (cv2.bitwise_or(magenta_img[:, :, 2], yellow_img[:, :, 2]))
    blue_img[:, :, 0]   = B - (cv2.bitwise_or(cyan_img[:, :, 0], magenta_img[:, :, 0]))
    green_img[:, :, 1]  = G - (cv2.bitwise_or(yellow_img[:, :, 1], cyan_img[:, :, 1]))

    cv2.imshow("Red", red_img)
    cv2.imshow("Blue", blue_img)
    cv2.imshow("Green", green_img)
    cv2.imshow("Yellow", yellow_img)
    cv2.imshow("Cyan", cyan_img)
    cv2.imshow("Magenta", magenta_img)

    BGR = cv2.merge([B, G, R])

    cv2.waitKey(0)
    rospy.spin
    cv2.destroyAllWindows()

if __name__ == '__main__':
    pure_color_extractor()