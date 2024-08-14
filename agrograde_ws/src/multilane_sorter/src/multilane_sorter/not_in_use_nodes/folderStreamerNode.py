#!/usr/bin/env python3

import rospy
import sys
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
import time
from uuid import uuid4
from utils.dateTimeLocUtils import DateTimeLocation
import os
from datetime import datetime
dtl = DateTimeLocation(defalut_storage="") 


bridge = CvBridge()
rospy.init_node('folder_streamer_node',anonymous=True)
rospy.loginfo('folder_streamer_node  started')
ir_clock_pub = rospy.Publisher('/'+os.environ['HOME'].split('/')[2]+'/ir_clock_channel', Int16MultiArray, queue_size=1)
img_pub = rospy.Publisher('/'+os.environ['HOME'].split('/')[2]+'/image', Image, queue_size=1)


folder_path = "/home/'+os.environ['HOME'].split('/')[2]+'/Downloads/test_images/"

ir_pulse_pub = rospy.Publisher('/'+os.environ['HOME'].split('/')[2]+'/ir_pulse_channel', Int16MultiArray, queue_size=1)



def start_node():
    k = 0 

    while not rospy.is_shutdown():

        for i in os.listdir(folder_path):
            pulse_start = time.time()
            frame = cv2.imread(os.path.join(folder_path,i))

            pulse_delta = int((time.time() - pulse_start)*1000)

            pulse = Int16MultiArray()
            pulse.data = [pulse_delta]
            ir_pulse_pub.publish(pulse)  

            arr = Int16MultiArray()
            arr.data = [k]
            ir_clock_pub.publish(arr)
            rospy.logdebug(arr)

            msg_frame = bridge.cv2_to_imgmsg(frame,encoding="bgr8")
            msg_frame.header.seq = k
            msg_frame.header.stamp = rospy.rostime.get_rostime()
            # msg_frame.header.frame_id = str(time.strftime("%Y%m%d-%H%M%S%f"))
            msg_frame.header.frame_id = str(datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
            
            img_pub.publish(msg_frame)
            k+=1

            rospy.Rate(8).sleep()




if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
