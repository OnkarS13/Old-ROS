#!/usr/bin/env python3
"""
Author: Vasan Naddoni
copyright @ Occipital Technologies Pvt Ltd
"""

import rospy
import os
from multilane_sorter.msg import CustomMessage1 as processingMsg
from multilane_sorter.msg import Signal as signal_message
from std_msgs.msg import Int16MultiArray, String
from database.databasetask import database_arch
from utils.settings import CommoditySettings
from sensor_msgs.msg import Image
from utils.dateTimeLocUtils import DateTimeLocation
from utils.dataRecorder import BlackBox
import json
dtm = DateTimeLocation()

class Database(object):
    def __init__(self):
        self.multiplier = float(rospy.get_param('sortobot/multiplier/'+os.environ['HOME'].split('/')[2]))
        self.did = int(rospy.get_param('sortobot/config/orderID'))
        self.commodity = str(rospy.get_param('sortobot/config/commodity'))
        self.commodity_settings = CommoditySettings(self.commodity)
        self.length  = 0
        self.database = database_arch()
        self.color_dict = dict((v,k) for k,v in rospy.get_param('sortobot/config/settings/'+self.commodity+'/color_dict').items())

    def get_defect_array(self,msg):
        data_string = msg.data
        self.def_array = data_string.strip('][').split(', ')
        self.defect_array = [float(i) for i in self.def_array] 

        self.get_defect_class(self.defect_array)
        # print(self.defect_array,type(self.defect_array))

    def get_defect_class(self,def_list):
        max_val = max(def_list[1:])
        index_val = def_list.index(max_val)

        if index_val  == 1 and float(max_val) > 5 :
            self.defect = "Black smut"
        elif index_val == 2 and float(max_val) > 5 :
            self.defect = "Peeled"
        elif index_val == 3 and float(max_val) > 5 :
            self.defect = "Sprouts"
        elif index_val == 4 and float(max_val) > 5 :
            self.defect = "Rotten"
        else:
            self.defect = "Normal"

    def cameraNode(self,msg):
        # print(msg.cam_frame_id)
        self.node_cam = {
            "cam_frame_id" : str(msg.header.frame_id), 
            "cam_stamp" : str(msg.header.stamp),
            "cam_seq" : str(msg.header.seq)
        }
    def processingNode(self,msg):
        self.length = int(msg.x*float(rospy.get_param('sortobot/multiplier/'+os.environ['HOME'].split('/')[2])))

        
        self.node_processing = {
            "proc_frame_id" : str(msg.header.frame_id) ,
            "proc_stamp" : str(msg.header.stamp) ,
            "proc_seq" : str(msg.header.seq)  , 
            
            "pred_x" : msg.x,
            "pred_y" : msg.y,

            "pred_prior_x" : msg.prior_x,
            "pred_prior_y" : msg.prior_y,

            "length_predicted" : str(msg.x*float(rospy.get_param('sortobot/multiplier/'+os.environ['HOME'].split('/')[2]))),
            "breadth_predicted" : str(msg.y*float(rospy.get_param('sortobot/multiplier/'+os.environ['HOME'].split('/')[2]))),

            "display_size" : str(msg.x*float(rospy.get_param('sortobot/multiplier/'+os.environ['HOME'].split('/')[2]))),

            "display_defect" : self.defect,
            "defect_array" : self.def_array,
            "defect_code_predicted" : msg.defect_code,
            "color_code_predicted" : msg.color_code,
            "display_color" : self.color_dict[msg.color_code] if msg.color_code != 100 else "None",
        }

    def decisionNode(self,msg):
        self.node_decision = {
            "decision_frame_id" : str(msg.header.frame_id) ,
            "decision_seq" : str(msg.header.seq) ,
            "decision_stamp" : str(msg.header.stamp),
            "decision_a" : msg.ac_a,
            "decision_b" : msg.ac_b ,
            "decision_c" : msg.ac_c 
        }

    def get_cleat_status(self,msg):
        self.cleat_status = msg.data

    def get_cleat_timings(self,msg):
        self.cleat_time = int(msg.data[0])

    def get_cleat_diagnostics(self,msg):
        self.cleat_diagnostics = msg.data

    def signalNode(self,msg):
        self.node_signal = {
            "output_signal_frame_id" : str(msg.header.frame_id) ,
            "output_signal_seq" : str(msg.header.seq) ,
            "output_signal_stamp" : str(msg.header.stamp) ,
            "output_signal_a" : msg.ac_a ,
            "output_signal_b" : msg.ac_b ,
            "output_signal_c" : msg.ac_c 
        }

        if(self.length != 0):
            data_dynamodb = {
                "day" : str(dtm.getTimeForDB().day),
                "month" : str(dtm.getTimeForDB().month),
                "year": str(dtm.getTimeForDB().year),
                "order_id": str(rospy.get_param("/sortobot/config/orderID")),
                "query_flag": False,
                **self.node_cam,
                **self.node_processing,
                **self.node_decision,
                **self.node_signal,
                "cleat_status":self.cleat_status,
                "cleat_timings": self.cleat_time,
                "cleat_diagnostics": self.cleat_diagnostics
            }
            self.database.store_data(data_dynamodb)

db = Database() 


def start_node():
    rospy.init_node('database_node')
    
    rospy.loginfo('database node started')

    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/image', Image, db.cameraNode)
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/prediction_data_channel',processingMsg,db.processingNode)
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/decision_assignment_channel',signal_message,db.decisionNode)
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/outbound_signal_channel',signal_message,db.signalNode)
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/defect_array_channel',String,db.get_defect_array)
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/error_correction_channel',String,db.get_cleat_status)
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/ir_pulse_channel', Int16MultiArray,db.get_cleat_timings)
    rospy.Subscriber('/'+os.environ['HOME'].split('/')[2]+'/cleat_diagnostics',String,db.get_cleat_diagnostics)
    rospy.spin()


if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass