#!/usr/bin/env python3

import glob
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
import skimage
from skimage import io
import os

class ImageLoad(object):
    def __init__(self):
        rospy.init_node('image_publisher')
        self.pub = rospy.Publisher('image_raw', Image, queue_size = 1)
        self.rate =rospy.Rate(1)
        self.bridge = CvBridge()

    def image_loop(self):  
        imgs = glob.glob("/home/agrograde/agrograde_ws/src/multilane_sorter/assets/images/lane_1/camera_11/*.jpg")
        while not rospy.is_shutdown():
            try:
                for img in imgs:
                    self.n= io.imread(img)
                    self.n= self.bridge.cv2_to_imgmsg(self.n,encoding='rgb8')
                    self.pub.publish(self.n)
                    rospy.sleep(5)   
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
  
                      

if __name__ == '__main__':
    node = ImageLoad()
    node.image_loop() 
    rospy.spin()       