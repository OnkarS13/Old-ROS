#!/usr/bin/env python3
"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""

import time
import os,sys
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray, String
import random 



rospy.init_node('pulsar_tester',anonymous=True)
# rospy.loginfo('camera_node  started')
ir_pulse_pub = rospy.Publisher('/'+os.environ['HOME'].split('/')[2]+'/ir_pulse_channel', Int16MultiArray, queue_size=1)

delays = [random.randint(280, 360) for i in range(97)]

delays[9]   = 121
delays[11]  = 620
delays[15]  +=250
delays[25]  +=150
delays[35]  -=150

# print(delays)
while True:
    for i in delays:
        k = Int16MultiArray()
        k.data =[i]
        ir_pulse_pub.publish(k)
        print(i)
        time.sleep(.01)