#!/usr/bin/env python3


import cv2, rospy
from sensor_msgs.msg import Image
import base64
from cv_bridge import CvBridge, CvBridgeError
import subprocess


bridge = CvBridge()	

def image_callback(msg):
    """try:
        cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
    except CvBridgeError as e:
        print(e)"""
    try:
        # Convert Image message to base64-encoded string
        image_data = base64.b64encode(msg.data)

	    # Create the cURL command with the base64-encoded image data
        curl_command = 'base64 image_data | curl -d @- "http://localhost:9001/dripbot2.0/6?api_key=qd9UzPOFO4zO31AOCT37"'

	    # Execute the cURL command using subprocess
        subprocess.Popen(curl_command, stdin=subprocess.PIPE, shell=True).communicate(input=image_data)

    except Exception as e:
        rospy.logerr('Error processing the image: {}'.format(str(e)))

def main():
    rospy.init_node('image_processor_node', anonymous=True)
    rospy.Subscriber('/video_source/raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
