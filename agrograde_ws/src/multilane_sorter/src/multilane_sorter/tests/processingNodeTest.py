#!/usr/bin/env python3
"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String



bridge = CvBridge()


def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)


def start_node():
    rospy.init_node('processing_node_test')
    rospy.loginfo('processing_node_test started')



    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/prediction_data_channel', Int16MultiArray,process_data)
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/gui_img_channel', Image,process_image)

    rospy.spin()

def process_data(msg):
    print((msg.data))


def process_image(msg):
    try:
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")

    except Exception as err:
        print( err)
    
    # showImage(orig)

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass