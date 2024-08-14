#!/usr/bin/env python3

import rospy
import os,sys
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.abspath(rospy.get_param('sortobot/config/relative_path_agro_1')))

from std_msgs.msg import Int16MultiArray


class CleatSensonMonitor(object):
    def __init__(self) :
        self.x = []
        self.y = []
        self.count = 0
        

    def getPlotData(self,msg):
        
        self.x.append(int(msg.data[0]))
        self.y.append(self.count)
        print(self.x,"\n",self.y)
        plt.scatter(self.x,self.y, color='green')
        #     plt.scatter(x_red,y_red, color='red')
        #     plt.xticks([n for n in range(0,len(df),100)])   #can change the values as required
        #     plt.yticks([r for r in range(0,500,10)])    #can change the values as required
        plt.axhspan(200,400, color='green', alpha=0.5)   # +- 10 of 270
        plt.xlabel("frame no.")
        plt.ylabel("cleat Timings")
        plt.legend(labels=["accepted_range", "inrange","outrange"])
        plt.title("Cleat timings lane ")
        
        plt.show()
        self.count+=1

csm = CleatSensonMonitor()

def start_node():  
    rospy.init_node('liveCleatSensor_node',anonymous=True)
    rospy.loginfo('liveCleatSensor_node  started')


    rospy.Subscriber('/ir_pulse_channel', Int16MultiArray,csm.getPlotData)

    # if rospy.get_param("/sortingTrigger"):
    #     csm.x = []
    #     csm.y = []
    #     csm.count = 0

    rospy.spin()


if __name__ == "__main__":
    start_node()