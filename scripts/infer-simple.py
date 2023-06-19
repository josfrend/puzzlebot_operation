#!/usr/bin/env python
# Import necessary libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np
import requests
import time


# Load config
ROBOFLOW_API_KEY = "qd9UzPOFO4zO31AOCT37"
ROBOFLOW_MODEL = "dripbot2.0/6"
ROBOFLOW_SIZE = 416

FRAMERATE = 30
BUFFER = 10

# Construct the Roboflow Infer URL
# (if running locally replace https://detect.roboflow.com/ with eg http://127.0.0.1:9001/)
upload_url = "".join([
    "http://localhost:9001/",
    ROBOFLOW_MODEL,
    "?api_key=",
    ROBOFLOW_API_KEY,
    "&format=image",
    "&stroke=5"
])
# Initialize ROS node
rospy.init_node('roboflow_node')

# Create CvBridge object
bridge = CvBridge()

# Define callback function to process incoming images
def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    img = bridge.imgmsg_to_cv2(msg)
    # Resize (while maintaining the aspect ratio) to improve speed and save bandwidth
    scale_percent = 30
    width = int(img.shape[1] * scale_percent/100)
    height = int(img.shape[0] * scale_percent/100)
    dim = (width,height)


    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    

    # Encode image to base64 string
    retval, buffer = cv2.imencode('.jpg', resized)
    img_str = base64.b64encode(buffer)

    # Get prediction from Roboflow Infer API
    resp = requests.post(upload_url, data=img_str, headers={
        "Content-Type": "application/x-www-form-urlencoded"
    }, stream=True).raw

    # Parse result image
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)

    # Display the inference results

    #processed_image.publish(bridge.cv2_to_imgmsg(imageDecoded))
    cv2.imshow('image',image)
    cv2.waitKey(1)



# Create subscriber to /video_source/raw topic
rospy.Subscriber('/video_source/raw', Image, image_callback)
processed_image = rospy.Publisher('/img_detection', Image, queue_size=10)
# Start the main loop
while not rospy.is_shutdown():
    rospy.spin()

# Release resources when finished
cv2.destroyAllWindows()
