#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np
import requests
import time
import threading
from std_msgs.msg import String


clase = ""
# Load config
ROBOFLOW_API_KEY = "qd9UzPOFO4zO31AOCT37"
ROBOFLOW_MODEL = "dripbot2.0/8"
ROBOFLOW_SIZE = 416

# Construct the Roboflow Infer URL
upload_url = "".join([
    "https://detect.roboflow.com/",
    ROBOFLOW_MODEL,
    "?api_key=",
    ROBOFLOW_API_KEY,
    "&format=json",
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
    scale_percent = 15
    width = int(img.shape[1] * scale_percent/100)
    height = int(img.shape[0] * scale_percent/100)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

    # Encode image to base64 string
    retval, buffer = cv2.imencode('.jpg', resized)
    img_str = base64.b64encode(buffer)

    # Start a new thread to handle the API request
    threading.Thread(target=process_image, args=(img_str,)).start()

def process_image(img_str):
    # Get prediction from Roboflow Infer API
    data = requests.post(upload_url, data=img_str, headers={"Content-Type": "application/json"}, stream=True).json()

    # Extracting class and confidence values
    predictions = data['predictions']
    if predictions:
        prediction = predictions[0]
        class_value =prediction['class']
        confidence_value = prediction['confidence']
        if confidence_value > 0.45:
            if class_value == "1":
                clase = "roadworks"
            elif class_value == "2":
                clase = "stop"
            elif class_value == "4":
                clase = "right"
            elif class_value == "5":
                clase = "left"
            elif class_value == "6":
                clase = "straight"
            elif class_value == "greenlight":
                clase = "green"
            elif class_value == "redlight":
                clase = "red"
            elif class_value == "greenlight":
                clase = "yellow"
            else:
                clase = " "
        else:
            clase = " "

        # Printing the extracted values
        rospy.loginfo("Class: %s", class_value)
        rospy.loginfo("Confidence: %s", confidence_value)
    else:
        clase = " "

    # Publish the inference results
    signals.publish(clase)

# Create subscriber to /video_source/raw topic
rospy.Subscriber('/video_source/raw', Image, image_callback)
signals = rospy.Publisher('/sign', String, queue_size=10)

# Start the main loop
rospy.spin()

# Release resources when finished
cv2.destroyAllWindows()

