"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""
import sys, os
sys.path.insert(0, os.path.abspath('..'))

from serial import Serial
import numpy as np
import time
from collections import deque
import rospy



class RelaySignal(object):

    def __init__(self,arduino_num,mode):
        self.mode = mode
        # arduino_num = rospy.get_param('~arduino_num')
        if self.mode:
            self.ser = Serial()
            self.ser.baudrate = 115200
            self.ser.port = "/dev/ttyUSB_LANE"+str(arduino_num) #/dev/ttyUSB_LANE1
            # self.ser.port = "/dev/ttyACM"+str(arduino_num)
            # print(self.ser.port)

            self.ser.timeout = 0.1
            self.ser.open()
            time.sleep(2)
            self.ser.reset_input_buffer()

    def relay(self,signal):
        
        self.ser.write(bytearray(signal))
        rospy.loginfo("publishing to arduino")

    def close(self):
        if self.mode:
            self.ser.close()

class ActuationDevices(object):

    def __init__(self,t1=1,t2=1,t3=1):
        # set the trigger max len using the distance data from the config
        self.trigger1 = deque([0]*t1,maxlen=t1)
        self.trigger2 = deque([0]*t2,maxlen=t2)
        self.trigger3 = deque([0]*t3,maxlen=t3)
 
