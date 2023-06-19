#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

cont = 1

def camera_callback(msg): 
    global cont  
    global cv_image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    # Resize the image to 640x480
    image = cv2.resize(image, (640, 480))
    cv2.imwrite("src/puzzlebot_operation/images/image" + str(cont) + ".jpg", image)
    cont+=1
    print("saved")

# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("Saving Pictures")

    # Initialize and Setup node at 100Hz
    rospy.init_node("fotitos")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    rospy.Subscriber("video_source/raw", Image, camera_callback)

    cont2 = 0

    #Run the node
    while not rospy.is_shutdown():
       cont2+=1