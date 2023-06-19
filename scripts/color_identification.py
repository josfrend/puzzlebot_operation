#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
import cv2 as cv
import numpy as np
import argparse


class colorIdentificator():
    def __init__(self):
        # Define Publishers and Subscibers
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.red_pub = rospy.Publisher('/red_traffic_light', Image, queue_size=10)
        self.yellow_pub = rospy.Publisher('/yellow_traffic_light', Image, queue_size=10)
        self.green_pub = rospy.Publisher('/green_traffic_light', Image, queue_size=10)
        self.light_pub = rospy.Publisher('/light_color', String, queue_size=10)
        self.light_color = 'n'
        self.bridge = CvBridge()
        rospy.on_shutdown(self.stop)

        #Define threshold values for traffic light detection
        self.cv_image = None
        self.red_low_H = 147
        self.red_low_S = 100
        self.red_low_V = 152
        self.red_high_H = 180
        self.red_high_S = 255
        self.red_high_V = 255

        self.red2_low_H = 0
        self.red2_low_S = 5
        self.red2_low_V = 0
        self.red2_high_H = 210
        self.red2_high_S = 152
        self.red2_high_V = 255

        self.yellow_low_H = 50
        self.yellow_low_S = 30
        self.yellow_low_V = 80
        self.yellow_high_H = 65
        self.yellow_high_S = 60
        self.yellow_high_V = 95

        self.green_low_H = 32
        self.green_low_S = 146
        self.green_low_V = 117
        self.green_high_H = 86
        self.green_high_S = 255
        self.green_high_V = 244

        self.green2_low_H = 66
        self.green2_low_S = 122
        self.green2_low_V = 141
        self.green2_high_H = 91
        self.green2_high_S = 255
        self.green2_high_V = 255

        # Define erosion and dilation parameters for blob detection
        self.erosion_size = 7
        self.dilatation_size = 0
        self.max_elem = 2
        self.max_kernel_size = 21

        # Define SimpleBlobDetector parameters
        self.params = cv.SimpleBlobDetector_Params()
        self.params.filterByCircularity = True
        self.params.minCircularity = 0.65
        self.params.filterByArea = True
        self.params.minArea = 8
        self.params.maxArea = 500000
        self.params.filterByInertia = True
        self.params.minInertiaRatio = 0.6
        self.params.filterByConvexity = True
        self.params.minConvexity = 0.5

        # Create SimpleBlobDetector object
        self.detector = cv.SimpleBlobDetector_create(self.params)

        # Counters
        self.red_count = 0
        self.yellow_count = 0
        self.green_count = 0

    def blobs(self,val):
        # Setup of dilatation and erosion elements
        element = cv.getStructuringElement(cv.MORPH_RECT, (2 * self.erosion_size + 1, 2 * self.erosion_size + 1),(self.erosion_size, self.erosion_size))
        erosion_dst = cv.erode(val, element)
        element = cv.getStructuringElement(cv.MORPH_RECT, (2 * self.dilatation_size + 1, 2 * self.dilatation_size + 1),(self.dilatation_size, self.dilatation_size))
        dilatation_dst = cv.dilate(erosion_dst, element)

        # Flip image to get detections
        dilatation_dst = 255-dilatation_dst

        # Use detector to get the keypoints
        keypoints = self.detector.detect(dilatation_dst)

        # Draw those keypoints in the frame
        blank = np.zeros((1, 1))
        blobs_d = cv.drawKeypoints(dilatation_dst, keypoints, blank, (255, 0, 0),cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv.putText(blobs_d, str(len(keypoints)), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        # Return image with indicators and a count of the blobs detected
        return blobs_d, len(keypoints)


    def image_callback(self,data):
        # Read image using the bridge
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        # Change color encoding and resize image
        frame_HSV = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        frame_HSV = cv.resize(frame_HSV, (680,480))

        # Apply a mask for each color 
        red_HSV = cv.inRange(frame_HSV, (self.red_low_H, self.red_low_S, self.red_low_V), (self.red_high_H, self.red_high_S, self.red_high_V))
        yellow_HSV = cv.inRange(frame_HSV, (self.yellow_low_H, self.yellow_low_S, self.yellow_low_V), (self.yellow_high_H, self.yellow_high_S, self.yellow_high_V))
        green_HSV = cv.inRange(frame_HSV, (self.green_low_H, self.green_low_S, self.green_low_V), (self.green_high_H, self.green_high_S, self.green_high_V))
        green2_HSV = cv.inRange(frame_HSV, (self.green2_low_H, self.green2_low_S, self.green2_low_V), (self.green2_high_H, self.green2_high_S, self.green2_high_V))
        green = cv.add(green_HSV, green2_HSV)

        # Send each image to the blob detector
        filtered_image, self.red_count = self.blobs(red_HSV)
        filtered_image2, self.yellow_count = self.blobs(yellow_HSV)
        filtered_image3, self.green_count = self.blobs(green)

        # Publish the result image
        self.red_pub.publish(self.bridge.cv2_to_imgmsg(filtered_image, "bgr8"))
        self.yellow_pub.publish(self.bridge.cv2_to_imgmsg(filtered_image2, "bgr8"))
        self.green_pub.publish(self.bridge.cv2_to_imgmsg(filtered_image3, "bgr8"))

    
    def run(self):    

        # Publish the color according to the relevance
        if(self.red_count > 0): self.light_color = 'r'
        elif(self.yellow_count > 0): self.light_color = "y"
        elif(self.green_count > 0): self.light_color = 'g'
        else: self.light_color = 'n'

        self.light_pub.publish(self.light_color)


    def stop(self):
        print("Stopping")

    

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("light_detector")
    color = colorIdentificator()
    rate = rospy.Rate(100)
    rospy.on_shutdown(color.stop)

    print("processing")
    try:
        while not rospy.is_shutdown():
            color.run()
            rate.sleep()

    except rospy.ROSInterruptException():
        pass
