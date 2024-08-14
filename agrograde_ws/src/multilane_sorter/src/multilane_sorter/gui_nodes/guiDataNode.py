#!/usr/bin/env python3
"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""
from multilane_sorter.msg import gui as guiMsg
from multilane_sorter.msg import Signal
import numpy as np 
import os,sys,cv2
import rospy
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml

from datetime import datetime



bridge = CvBridge()


class GuiDataUpdate(object):
    '''
    Node for the backend calculations for the GUI
    '''

    def __init__(self,lane_ls=None):
        rospy.init_node('gui_node')
        rospy.loginfo('guiData_node started')
        self.gui_msg = guiMsg()
        self.count = [0,0,0]
        self.lanes = lane_ls
        self.systemUp = datetime.now().replace(microsecond=0)
        self.sortingTriggerInstance = datetime.now().replace(microsecond=0)
        self.sortingTime = 0
        self.lane_id = 1
        self.lanes = lane_ls
        self.seq = 0
        self.calculatedTotalTP = 0
        self.positiveTP = 0
        self.negativeTP = 0
        # self.size = 0
        rospy.set_param("/gui/dump_param",0)
        rospy.set_param("/sortingTrigger",2)
        # self.estimateThroughput()
        self.gui_data_channel = rospy.Publisher('/guiData', guiMsg, queue_size=1)
        # rospy.Subscriber("/lane_1/decision_assign",Signal, self.updateCount)

    def updateCount(self,msg):
        if rospy.get_param("/gui/dump_param"):
            self.dumpParam()
            rospy.set_param("/gui/dump_param",0)
        self.seq+=1
        self.a,self.b,self.c = msg.ac_a,msg.ac_b,msg.ac_c
        data = [self.c,self.b,self.a]
        # self.count = np.sum([data,self.count],axis=0)
        self.count = [data[i] + self.count[i] for i in range(len(data))]
        self.gui_msg.actuatorStats = str(self.count)
        self.calculateUptime()
        # self.size = str(msg.size)

    def dumpParam(self):
        parameters_path = os.path.join(rospy.get_param("/sortobot/config/relative_path_dev"),"yamls/parameters.yaml")
        decision_path = os.path.join(rospy.get_param("/sortobot/config/relative_path_dev"),"yamls/decision.yaml")
        doc = rospy.get_param("/sortobot")
        full = {'sortobot':doc}
        with open(parameters_path, 'w') as f:
            data = yaml.dump(full, f)
        doc_1 = rospy.get_param("/decisionParams")
        full_1 = {'decisionParams':doc_1}
        with open(decision_path, 'w') as f:
            data = yaml.dump(full_1, f)
            
    def calculateUptime(self):
        delta_st = '0:00:00'
    
        if int(rospy.get_param("/sortingTrigger")) == 1:
            #software reset
            self.sortingTriggerInstance = datetime.now().replace(microsecond=0)
            self.positiveTP = 0
            self.negativeTP = 0
            self.calculatedTotalTP = 0
            self.count = [0,0,0]
            rospy.set_param("/sortingTrigger",0)

        if int(rospy.get_param("/sortingTrigger")) == 0:
            #continous calculation
            delta_st = datetime.now().replace(microsecond=0) - self.sortingTriggerInstance
            # wt = (self.size/13.23)**3/1000 #kg
            # self.positiveTP += wt*self.a + wt*self.b
            # self.negativeTP += wt*self.c

            # self.calculatedTotalTP = (self.positiveTP+self.negativeTP)


        self.gui_msg.sortingUptime = str(delta_st)

        if int(rospy.get_param("/sortingTrigger")) == 2:
            self.positiveTP = 0
            self.negativeTP = 0
            self.calculatedTotalTP = 0
            self.gui_msg.sortingUptime = str(delta_st)
        
        delta_su = datetime.now().replace(microsecond=0) - self.systemUp
        self.gui_msg.systemUptime = str(delta_su)
        
        # if self.seq%50:
        # self.estimateThroughput()
        
        self.publish()
    
    # def estimateThroughput(self):
    #     total_tp = max(3.12,2.5)

    #     self.gui_msg.throughputOverAll = str(round(total_tp,2))
    #     self.gui_msg.throughputAccepted= str(round(self.positiveTP*100/(self.calculatedTotalTP+0.1),2))
    #     self.gui_msg.throughputRejected= str(round(self.negativeTP*100/(self.calculatedTotalTP+0.1),2))

    # def dumpParams(self):
    #     if rospy

    def publish(self):
        self.gui_data_channel.publish(self.gui_msg)


lane_ls = ['lane_1','lane_2','lane_3','lane_4']
gdu = GuiDataUpdate(lane_ls)

def start_node():

    rospy.Subscriber('/lane_1/decision_assign', Signal,gdu.updateCount)
    rospy.Subscriber('/lane_2/decision_assign', Signal,gdu.updateCount)
    rospy.Subscriber('/decision_assign', Signal,gdu.updateCount)
    rospy.Subscriber('/lane_4/decision_assignment_channel', Signal,gdu.updateCount)
    
    gdu.publish()

    rospy.spin()



if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        start_node()
    






