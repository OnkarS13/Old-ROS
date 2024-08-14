#!/usr/bin/env python3

"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""

import sys, os
sys.path.insert(0, os.path.abspath('..'))

from ai.combined_model import CombinedModel
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
ai_model ="/home/panick/sortobot/src/sorter/scripts/assets/Onion_machine_COMNet_v2.2.0.h5"
model = CombinedModel((256,256),(400,400),(300,300),ai_model)

def showImage(img):
    cv2.namedWindow("image",cv2.WINDOW_NORMAL)
    cv2.imshow('image', img)
    cv2.waitKey(1)


def start_node():
    rospy.init_node('frameGrabber')
    rospy.loginfo('frameGrabber node started')
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/image', Image, process_image)
    rospy.spin()

def process_image(msg):
    try:
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        model.predict_quantities(orig)
        mask = model.segmented_mask
        # print(mask)
        showImage(mask)
        # edges = cv2.Canny(drawImg[:,:,1],50,255)

    except Exception as err:
        print( err)
    


start_node()
