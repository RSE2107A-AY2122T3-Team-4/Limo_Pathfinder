#!/usr/bin/env python

# ROS Python
from pickle import NONE
import rospy

# OpenCV2
import cv2 as cv

# NumPy
import numpy as np

# Image
from sensor_msgs.msg import Image

# CvBridge
from cv_bridge import CvBridge

# Twist
from geometry_msgs.msg import Twist

# Publisher defined in Global Namespace
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Gaussian Blur
def gaussian_blur(image,
                  kernel_size = 13):
    """
    Takes an input of an Image.
    Applies a Gaussian Blur to the image.
    Returns the transformed Image.
    """

    return cv.GaussianBlur(image, (kernel_size, kernel_size), 0)

# Canny Detector
def canny_detector(image, 
                   low_threshold = 50,
                   high_threshold = 150):
    """
    Takes an input of an Image.
    Applies the Canny Edge Detection algorithm to the Image.
    Returns the result.
    """

    return cv.Canny(image, low_threshold, high_threshold)

# Lane Region Selector
def lane_region_selector(image):
    """
    Takes an input of an Image.
    Applies a Region of Interest to a defined space.
    Returns the adjusted Image.
    """

    mask = np.zeros_like(image)
    rows, cols = image.shape[:2]
    bottom_left  = [cols * 0.00, rows * 1.00]
    top_left     = [cols * 0.05, rows * 0.40]
    bottom_right = [cols * 1.00, rows * 1.00]
    top_right    = [cols * 0.95, rows * 0.40]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    cv.fillPoly(mask, vertices, 255)

    return cv.bitwise_and(image, mask)

# Track Region Selector
def track_region_selector(image):
    """
    Takes an input of an Image.
    Applies a Region of Interest to a defined space.
    Returns the adjusted Image.
    """

    mask = np.zeros_like(image)
    rows, cols = image.shape[:2]
    bottom_left  = [cols * 0.40, rows * 1.00]
    top_left     = [cols * 0.45, rows * 0.60]
    bottom_right = [cols * 0.60, rows * 1.00]
    top_right    = [cols * 0.55, rows * 0.60]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    cv.fillPoly(mask, vertices, 255)

    return cv.bitwise_and(image, mask)

# Hough Transform
def hough_transform(image,
                    rho = 1,
                    theta = np.pi / 180,
                    threshold = 50,
                    minLineLength = 15,
                    maxLineGap = 100):
    """
    Takes an input of an Image.
    Applies a Hough Line Transform to the image.
    Returns the transformed Image.
    """

    return cv.HoughLinesP(image,
                          rho = rho,
                          theta = theta,
                          threshold = threshold,
                          minLineLength = minLineLength,
                          maxLineGap = maxLineGap) 

# Define Average Line
def average_slope_intercept(lines):
    """
    Takes an input of hough lines
    Finds the gradient, y intercept and length of each hough line.
    Separates te lines into left and right lane based on the gradient of each line.
    Returns the average hough line for each side of the lane.
    """

    left_lines    = [] #(slope, intercept)
    left_weights  = [] #(length)
    right_lines   = [] #(slope, intercept)
    right_weights = [] #(length)

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 == x2:
                continue
            slope = ((y2 * 1.0) - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
            if slope < 0:
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))
    left_lane  = np.dot(left_weights,  left_lines) / np.sum(left_weights)  if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
    return left_lane, right_lane

# Pixel Points
def pixel_points(y1, y2, line):
    """
    Takes an input of a line's start and end point, and the gradient and y-intercept.
    Finds the x coordinate of the start and end point.
    Returns a a set of coordinates for the start and end point.
    """

    if line is None:
        return None
    slope, intercept = line
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    return ((x1, y1), (x2, y2))

# Define Lane Lines
def lane_lines(image, lines):
    """
    Takes an input of an Image and hough lines.
    Returns the average line for each side lane.
    """

    left_lane, right_lane = average_slope_intercept(lines)
    y1 = image.shape[0]
    y2 = y1 * 0.5
    left_line  = pixel_points(y1, y2, left_lane)
    right_line = pixel_points(y1, y2, right_lane)

    return left_line, right_line

# Insert Text
def insert_text(image, 
                text = 'Empty', 
                org = (0, 0), 
                font = cv.FONT_HERSHEY_SIMPLEX, 
                fontScale = 0.8, 
                color = [0, 0, 255],
                thickness = 2,
                lineType = cv.LINE_AA,
                bottomLeftOrigin = False
                ):
    """
    Takes an input of an Image, String and a Coordinate.
    Returns the image with text.
    """
    
    return cv.putText(image, text, org, font, fontScale, color, thickness, lineType, bottomLeftOrigin)

# Reader
def reader(data):
    """
    Reads an image of a road lane and shows where the lane lines are.
    """

    # Declaration and Initialisation of conversion variable
    bridge = CvBridge()

    # Source Image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    # Declaration and Initialisation of geometry_msgs/Twist variable
    vel_msg = Twist()
    vel_msg.linear.x  = 0
    vel_msg.linear.y  = 0
    vel_msg.linear.z  = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    # Make a copy of the Image for transformation
    result = np.copy(image)

    # Convert the input image to HSV
    converted_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    
    # Define threshold for the color White
    lower_threshold = np.uint8([  0,   0, 210])
    upper_threshold = np.uint8([255,  30, 255])
    white_mask = cv.inRange(converted_image, lower_threshold, upper_threshold)

    cv.imshow("w", white_mask)
    cv.waitKey(1)
    white_mask = cv.bitwise_and(image, image, mask = white_mask)

    # Define threshold for the color Yellow
    lower_threshold = np.uint8([ 20, 50, 50])
    upper_threshold = np.uint8([ 40, 255, 255])
    yellow_mask = cv.inRange(converted_image, lower_threshold, upper_threshold)

    yellow_mask = cv.bitwise_and(image, image, mask = yellow_mask)

    # Convert Image to Grayscale
    w_gray = cv.cvtColor(white_mask, cv.COLOR_BGR2GRAY)
    y_gray = cv.cvtColor(yellow_mask, cv.COLOR_BGR2GRAY)

    # Apply Gaussian Blur
    w_blur = gaussian_blur(w_gray)
    y_blur = gaussian_blur(y_gray)

    # Apply Canny Detector Algorithm
    w_edges = canny_detector(w_blur)
    y_edges = canny_detector(y_blur)

    # Limit the Region of Interest
    w_region = track_region_selector(w_edges)
    y_region = lane_region_selector(y_edges)

    # Find Lane Lines
    w_hough = hough_transform(w_region)
    y_hough = hough_transform(y_region)

    # Initialise variables
    gradient = 0.0
    left_gradient = 0.0
    right_gradient = 0.0
    line_found_flag = False

    # Detect White lane lines
    if w_hough is not None:

        line_found_flag = True

        lines    = [] #(slope, intercept)
        weights  = [] #(length)

        for x1, y1, x2, y2 in w_hough:
            if x1 == x2:
                continue
            slope = ((y2 * 1.0) - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
            lines.append((slope, intercept))
            lines.append((length))
        w_line  = np.dot(weights,  lines) / np.sum(weights)  if len(weights) > 0 else None
        y1 = image.shape[0]
        y2 = y1 * 0.5
        line  = pixel_points(y1, y2, w_line)

        if line is not None:
            cv.line(result, line[0], line[1], [0, 0, 255], 5, cv.LINE_AA)
            gradient = ((line[1][1] * 1.0) - line[0][1]) / (line[1][0] - line[0][0])
            gradient_str = str(gradient)
            insert_text(result, gradient_str, line[1])

    # If unable to detect White lane lines, use Yellow lane lines instead
    elif y_hough is not None:

        line_found_flag = True

        # Draw lane lines
        left_line, right_line = lane_lines(result, y_hough)

        # Find Left lane line
        if left_line is not None:
            cv.line(result, left_line[0], left_line[1], [0, 0, 255], 5, cv.LINE_AA)
            left_gradient = ((left_line[1][1] * 1.0) - left_line[0][1]) / (left_line[1][0] - left_line[0][0])
            left_gradient_str = str(left_gradient)
            insert_text(result, left_gradient_str, left_line[1])

        # Find Right lane line
        if right_line is not None:
            cv.line(result, right_line[0], right_line[1], [0, 0, 255], 5, cv.LINE_AA)
            right_gradient = ((right_line[1][1] * 1.0) - right_line[0][1]) / (right_line[1][0] - right_line[0][0])
            right_gradient_str = str(right_gradient)
            insert_text(result, right_gradient_str, right_line[1])

        # Sum of Left and Right lane line gradients
        gradient = left_gradient + right_gradient

    # Calculate Linear and Angular Velocity based on Gradient
    if line_found_flag:
        if gradient > -0.5 and gradient < 0.5:
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = 0.0
        elif gradient > 0.5:
            vel_msg.linear.x = 0.1
            if gradient > 1.0:
                vel_msg.angular.z = 1.0
            else:    
                vel_msg.angular.z = gradient/2.0
        elif gradient < -0.5:
            vel_msg.linear.x = 0.1
            if gradient < -1.0:
                vel_msg.angular.z = -1.0
            else:
                vel_msg.angular.z = gradient/2.0
    else:
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0

    # Publish geometry_msgs/Twist to cmd_vel
    velocity_publisher.publish(vel_msg)

    cv.imshow("LANE LINES", result)

    cv.waitKey(1)

# Line Tracker
def line_tracker():
    # Declaration and Initialisation of Node
    rospy.init_node('limo_line_tracker_node', anonymous=True)
    rospy.Subscriber('/camera/rgb/image_raw', Image, reader)

    # Keeps node from exiting until the node has been shutdown.
    rospy.spin()

    # Closes all active OpenCV windows
    cv.destroyAllWindows()

# Main
if __name__ == '__main__':
    try:
        line_tracker()
    except rospy.ROSInterruptException:
        pass
