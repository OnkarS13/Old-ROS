#!/usr/bin/env python3

import glob # --> glob: Used to retrieve files/pathnames matching a specified pattern.
import rospy # --> rospy: The ROS Python client library, which allows you to write ROS nodes in Python.
from sensor_msgs.msg import Image
import cv2 # --> cv2: OpenCV library used for image processing tasks.
from cv_bridge import CvBridge # --> A ROS utility to convert between ROS Image messages and OpenCV images.
from sensor_msgs.msg import CompressedImage
from skimage import io # -->skimage.io: A module from the scikit-image library used for reading and writing images.


class ImageLoad(object):
    def __init__(self): # --> The constructor method initializes the ROS node, sets up the image publisher, and prepares the CvBridge for converting OpenCV images to ROS Image messages.
        rospy.init_node('image_publisher')
        self.pub = rospy.Publisher('image_raw', Image, queue_size=1) # --> A ROS publisher object that will publish messages of type Image on the topic 'image_raw'.
        self.rate =rospy.Rate(1)
        self.bridge = CvBridge() # --> CvBridge(): Initializes a CvBridge object, which is used to convert between ROS Image messages and OpenCV images.

    def image_loop(self):  
        imgs = glob.glob("/home/agrograde/agrograde_ws/src/multilane_sorter/assets/images/lane_2/camera_21/*.jpg") # --> Uses glob to get a list of all .jpg image files in the specified directory. This directory contains the images to be published.
        while not rospy.is_shutdown(): # --> A loop that runs until ROS is shut down. This ensures that the node keeps running until it is explicitly terminated.
            try:
                for img in imgs:
                    self.n = io.imread(img) # --> io.imread(img): Reads the image using skimage.io.imread.
                    self.n = self.bridge.cv2_to_imgmsg(self.n, encoding = 'rgb8') # --> Converts the OpenCV image to a ROS Image message. The encoding rgb8 indicates that the image is in 8-bit RGB format.
                    self.pub.publish(self.n) # --> Publishes the image message on the image_raw topic.
                    rospy.sleep(5)   # --> Pauses the loop for 5 seconds before publishing the next image. This creates a delay between consecutive image publications.
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!") # --> This block catches any ROS-specific exceptions (like node shutdown) and logs an error message.
  
                      

if __name__ == '__main__':
    node = ImageLoad() # -->  Instantiates the ImageLoad class, creating a new node.
    node.image_loop() # --> Calls the image_loop method, starting the image publishing loop.
    rospy.spin()       # --> Keeps the program alive, allowing callbacks to continue executing until the node is shut down.