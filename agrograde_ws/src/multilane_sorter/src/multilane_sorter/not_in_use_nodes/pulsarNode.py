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
import json
from collections import deque


error_correction_signal = rospy.Publisher('/'+os.environ['HOME'].split('/')[2]+'/error_correction_channel', String, queue_size=1)
cleat_diagnostics = rospy.Publisher('/'+os.environ['HOME'].split('/')[2]+'/cleat_diagnostics', String, queue_size=1)

ir_clock_pub = rospy.Publisher('/'+os.environ['HOME'].split('/')[2]+'/ir_clock_channel', Int16MultiArray, queue_size=1)




class Pulsar(object):
    
    def __init__(self):
        self.freq = 0 #frequency of missing the cleats per cycle
        self.period = 97 #number of total cleats
        
        self.std = None
        self.mean_delay = None
        self.tolerance_max  = 350
        self.tolerance_min  = 180

        self.bin = deque([],maxlen=self.period)
        self.ir_glitch = 0

        self.total_glitches_percycle = None
        self.cleat = 0

        self.readings = deque([],maxlen=self.period)
        self.frame_no = 0


    def read(self,msg):
        self.delay = int(msg.data[0]) #in ms
        self.bin.append(self.delay)
        self.analysis()


    def sendErrorCorrectionSignal(self,flag):
        
        if   flag==1:
            msg = "IR_MAX_FAIL"

        elif flag==0:
            msg = "IR_MIN_FAIL"

        elif flag==-1:
            msg = "NO_IR_FAIL"



        error_correction_signal.publish(msg)

    def analysis(self):
        self.cleat+=1
        
        if self.delay > self.tolerance_max:
            print("Cleat {} Fail delay:{} > tolerance {}".format(self.cleat,self.delay,self.tolerance_max))            
            self.sendErrorCorrectionSignal(1)
            self.bin.pop()

        elif self.delay < self.tolerance_min:
            print("Cleat {} Fail delay:{} < tolerance {}".format(self.cleat,self.delay,self.tolerance_min))    
            self.sendErrorCorrectionSignal(0)
            self.bin.pop()
        
        elif self.delay  in range(self.tolerance_min,self.tolerance_max):
            self.sendErrorCorrectionSignal(-1)
            
            
        self.sendRegulatedIRPulse(msg='fckt!')
        self.readings.append(self.delay)

        self.selfCorrect()

    def selfCorrect(self):

        if len(self.bin) > 10:
            self.freq = len([ i for i in self.readings if i not in range(int(self.tolerance_min),int(self.tolerance_max))])
            
        diagnose_str = str(" Current IR glitches at: {} cleats per cycle(97 cleats)".format(self.freq))
        cleat_diagnostics.publish(diagnose_str)



    def sendRegulatedIRPulse(self,msg):

        arr = Int16MultiArray()
        arr.data = [self.frame_no]
        ir_clock_pub.publish(arr)
        self.frame_no+=1



pulsar = Pulsar()

def start_node():

    rospy.init_node('pulsar_node_started')
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/ir_pulse_channel', Int16MultiArray,pulsar.read)
    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass