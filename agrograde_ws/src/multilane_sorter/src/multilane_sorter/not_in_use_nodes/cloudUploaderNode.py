#!/usr/bin/env python3

import time
import os,sys
sys.path.insert(0, os.path.abspath('..'))

import cv2
import numpy as np
import rospy

from utils.uploader import CloudUploader
from std_msgs.msg import String



rospy.init_node('cloudUploaderNode')
rospy.loginfo('cloudUploaderNode  started')

CloudUploader = CloudUploader()

def start_node():

    # ?if rospy.get_param("uploadSignal"):
    while not rospy.is_shutdown():
        CloudUploader.upload_on_cloud

    
    rospy.Rate(1).sleep()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass