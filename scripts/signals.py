#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
import cv2
import numpy as np
import argparse

class traffic_signs_detector():
    def __init__(self):
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.traffic_sign_pub = rospy.Publisher('/sign', String, queue_size=10)
        self.bridge = CvBridge()
        print("Opening node")

        # Define lower and upper bounds for blue and red color
        self.lower_blue = np.array([100, 50, 150])
        self.upper_blue = np.array([120, 255, 255])

        self.lower_red = np.array([150, 84, 110])
        self.upper_red = np.array([180, 255, 255])

        # Define maximum and minimum areas for each traffic sign
        self.min_circle_area = 3000
        self.max_circle_area = 6000

        self.min_triangle_area = 2000
        self.max_triangle_area = 5000

        self.min_octagon_area = 3000
        self.max_octagon_area = 6000

    def is_ellipse_similar_to_circle(self, ellipse, similarity_threshold = 0.65):
        # Calculate the ratio between the major and minor axes
        ratio = ellipse[1][0] / ellipse[1][1]
        
        # Compare the ratio with the similarity threshold
        if abs(ratio - 1) < (1 - similarity_threshold):
            return True
        else:
            return False

    def calculate_contrast(self, frame):
        
        # Calculate the mean pixel intensity
        mean_intensity = np.mean(frame)
        
        # Calculate the standard deviation of pixel intensities
        std_dev = np.std(frame)
        
        # Return the contrast value
        return std_dev / mean_intensity
    
    def detect_blue_ellipses(self, frame):
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask for blue color
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        
        # Apply the mask to the frame
        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Convert masked frame to grayscale
        gray = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Perform edge detection
        edges = cv2.Canny(blurred, threshold1=30, threshold2=100)
        
        # Find contours in the edge image
        _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Loop over the contours
        for contour in contours:
            # Fit an ellipse to the contour
            if len(contour) >= 5:
                ellipse = cv2.fitEllipse(contour)
                
                # Calculate the area of the fitted ellipse
                area = np.pi * (ellipse[1][0] / 2) * (ellipse[1][1] / 2)
                
                # Check if the ellipse area is above the minimum area threshold
                if area >= self.min_circle_area and area <= self.max_circle_area:
                    if self.is_ellipse_similar_to_circle(ellipse):
                        # Extract ellipse parameters
                        (x, y), (major_axis, minor_axis), angle = ellipse
                        
                        # Draw the ellipse on the frame
                        cv2.ellipse(frame, (int(x), int(y)), (int(major_axis/2), int(minor_axis/2)), angle, 0, 360, (0, 255, 0), 4)
                        
                        # Approximate the ellipse as a polygon
                        perimeter = cv2.arcLength(contour, True)
                        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
                        
                        # Draw the polygon on the frame
                        cv2.drawContours(frame, [approx], -1, (0, 0, 255), 2)
                        
                        # Calculate the start and end points for the lines
                        start_x = int(x - major_axis/2)
                        end_x = int(x + major_axis/2)
                        start_y = int(y - minor_axis/2)
                        end_y = int(y + minor_axis/2)
                        
                        # Draw the vertical and horizontal lines inside the ellipse
                        cv2.line(frame, (int(x), start_y), (int(x), end_y), (255, 0, 0), 2)
                        cv2.line(frame, (start_x, int(y)), (end_x, int(y)), (255, 0, 0), 2)
                        
                        # Divide the ellipse into four quadrants
                        top_left = gray[start_y:int(y), start_x:int(x)]
                        top_right = gray[start_y:int(y), int(x):end_x]
                        bottom_left = gray[int(y):end_y, start_x:int(x)]
                        bottom_right = gray[int(y):end_y, int(x):end_x]
                        
                        # Calculate the contrast for each quadrant
                        contrast_top_left = self.calculate_contrast(top_left)
                        contrast_top_right = self.calculate_contrast(top_right)
                        contrast_bottom_left = self.calculate_contrast(bottom_left)
                        contrast_bottom_right = self.calculate_contrast(bottom_right)
                        
                        # Print the contrast values
                        # print("Top Left Contrast:", contrast_top_left)
                        # print("Top Right Contrast:", contrast_top_right)
                        # print("Bottom Left Contrast:", contrast_bottom_left)
                        # print("Bottom Right Contrast:", contrast_bottom_right)

                        sign = 0
                        tolerance = 0.1

                        if sign != 1 and contrast_bottom_right < contrast_bottom_left and contrast_bottom_right < contrast_top_left and contrast_bottom_right < contrast_top_right:
                            print("Turn right")
                            sign = 1
                            return "right"

                        if sign != 2 and contrast_bottom_left < contrast_bottom_right and contrast_bottom_left < contrast_top_left and contrast_bottom_left < contrast_top_right:
                            print("Turn left")
                            sign = 2
                            return "left"

                        if sign != 3 and contrast_top_left+contrast_top_right > contrast_bottom_right+contrast_bottom_left and (contrast_bottom_right-tolerance == contrast_bottom_left or contrast_bottom_right == contrast_bottom_left-tolerance) and (contrast_top_right-tolerance == contrast_top_left or contrast_top_right == contrast_top_left-tolerance):
                            print("Go forward")
                            sign = 3
                            return "forward"
                        
    def detect_red_triangles_and_octagons(self, frame, hsv_frame):
        
        mask = cv2.inRange(hsv_frame, self.lower_red, self.upper_red)

        
        masked_frame = cv2.bitwise_and(frame, frame, mask=mask)


        edges = cv2.Canny(masked_frame, 100, 200)


        _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            approximation = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            area = cv2.contourArea(approximation)

            if len(approximation) == 3:
                if area > self.min_triangle_area and area < self.max_triangle_area:
                    print("Slow down")
                    return "work"

            if len(approximation) == 5:
                if area > self.min_octagon_area and area < self.max_octagon_area:
                    print("Stop")
                    return "stop"

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            print("On callback")
        except CvBridgeError as e:
            print(e)
            return

        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        instruction = self.detect_red_triangles_and_octagons(frame, hsv)
        instruction = self.detect_blue_ellipses(hsv)

        self.traffic_sign_pub.publish(instruction)

    def stop(self):
        print('Stopping')
    
if __name__ == "__main__":
    #Initialise and Setup node
    rospy.init_node("traffic_signs_detector")
    rate = rospy.Rate(100)
    detector = traffic_signs_detector()
    rospy.on_shutdown(detector.stop)

    try:
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException():
        pass
