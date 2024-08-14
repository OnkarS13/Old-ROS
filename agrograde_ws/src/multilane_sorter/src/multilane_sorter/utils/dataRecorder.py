#!/usr/bin/env python3

"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""
import time
import os,sys
sys.path.insert(0, os.path.abspath('..'))

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from multilane_sorter.msg import CustomMessage1 as processingMsg
from multilane_sorter.msg import Signal as signal_message
from std_msgs.msg import Int16MultiArray, String
import json
from utils.dateTimeLocUtils import DateTimeLocation
from database.databasetask import database_arch as database

user = os.environ['HOME'].split('/')[2]
name = str(user.split('_')[1])

dtl = DateTimeLocation(defalut_storage=os.path.join(rospy.get_param('sortobot/config/relative_path_'+name),"local_data")) 
dtl_m = DateTimeLocation(defalut_storage=os.path.join(rospy.get_param('sortobot/config/relative_path_'+name),"metadata")) 


class BlackBox():
    def __init__(self):
        
        self.metadata_path = "./metadata"

        self.bridge = CvBridge() 

        self.cam_log = "initializing,-1"
        self.proc_frame_id = "initializing,-1"
        self.old_img = None
        self.cam_img = None


    def img_writer(self,img,name):
        
        # if rospy.get_param('sortobot/gui/status'):
        _,location = dtl.getLocation()

        # print(location)
        rospy.set_param('sortobot/config/img_dump_location',os.path.basename(os.path.normpath(str(location))))
        path = os.path.join(location,name+".jpg")
        cv2.imwrite(path,img)
        
        _,location_meta = dtl_m.getLocation()
        path_m = os.path.join(location_meta,name+".jpg")
        img_m = cv2.resize(img, (0,0), fx=0.3, fy=0.3)
        cv2.imwrite(path_m,img_m)
        # print(path)



    def getImg(self,msg):
        # would only work for cv_bridge based images
        try:
            return self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return self.bridge.imgmsg_to_cv2(msg, "8UC1")


    def defect_img(self,msg):

        self.def_img = self.getImg(msg)
        self.def_frame_id = "def_"+str(msg.header.frame_id) 
        self.def_stamp = msg.header.stamp
        self.def_seq = msg.header.seq
        print(self.def_frame_id)
        self.img_writer(self.def_img,self.def_frame_id)



    def cameraNode(self,msg):
        # print("-"*25)
        if self.cam_img is not None:
            self.old_img = self.cam_img

        self.cam_img = self.getImg(msg)
        self.cam_frame_id = msg.header.frame_id 
        self.cam_stamp = msg.header.stamp
        self.cam_seq = msg.header.seq
        
        self.cam_log = "cam_log, "+str(self.cam_seq)+", "+str(self.cam_frame_id)+", "+str(self.cam_stamp)
        rospy.loginfo(self.cam_log)


    def img_write_eligibity(self,msg):
        # info_array = json.load(msg.data)
        data_string = msg.data
        self.info_array = data_string.strip('][').split(', ')
        print("Measurement: ",float(self.info_array[0]))
        if float(self.info_array[0]) > 20:
            self.img_writer(self.old_img,self.cam_frame_id)

    def processingNode(self,msg):
        
        self.proc_frame_id = msg.header.frame_id
        self.proc_stamp  = msg.header.stamp
        self.proc_seq   = msg.header.seq
        
        self.x=msg.x
        self.y=msg.y

        self.prior_x=msg.prior_x
        self.prior_y=msg.prior_y

        self.defect = msg.defect_code 
        self.color  = msg.color_code

        proc_log_header = "proc_log, "+str(self.proc_seq)+", "+str(self.proc_frame_id)+", "+str(self.proc_stamp)
        proc_log_payload= str(self.x)+", "+str(self.y)+", "+str(self.prior_x)+", "+str(self.prior_y)+", "+str(self.defect)+", "+str(self.color)

        proc_log =proc_log_header+", "+proc_log_payload
        rospy.loginfo(proc_log)




    def decisionNode(self,msg):
        # will record generated signal

        # header
        self.dcsn_frame_id = msg.header.frame_id
        self.dcsn_seq   =  msg.header.seq
        self.dcsn_stamp =  msg.header.stamp
        
        # payload
        self.dcsn_a = msg.ac_a
        self.dcsn_b = msg.ac_b
        self.dcsn_c = msg.ac_c
        
        decision_log_header = "gen_sig_log, "+str(self.dcsn_seq)+", "+str(self.dcsn_frame_id)+", "+str(self.dcsn_stamp)
        decision_log_payload= str(self.dcsn_a)+", "+str(self.dcsn_b)+", "+str(self.dcsn_c)

        decision_log = decision_log_header+", "+decision_log_payload
        rospy.loginfo(decision_log)

    

    def signalNode(self,msg):
        # will record relayed signal

        # header
        self.output_signal_frame_id = msg.header.frame_id
        self.output_signal_seq   =  msg.header.seq
        self.output_signal_stamp =  msg.header.stamp
        
        # payload
        self.output_signal_a = msg.ac_a
        self.output_signal_b = msg.ac_b
        self.output_signal_c = msg.ac_c
        
        output_signal_log_header = "out_sig_log, "+str(self.output_signal_seq)+", "+str(self.output_signal_frame_id)+", "+str(self.output_signal_stamp)
        output_signal_log_payload= str(self.output_signal_a)+", "+str(self.output_signal_b)+", "+str(self.output_signal_c)

        output_signal_log = output_signal_log_header+", "+output_signal_log_payload
        rospy.loginfo(output_signal_log)
        # self.dump_data()

    

    def guiImg(self,msg):
        path = os.path.join(self.metadata_path,"processed_img")
        name = self.proc_frame_id+".jpg"
        self.cropped_img = self.getImg(msg)
        cv2.imwrite(os.path.join(path,name),self.cropped_img)
    
    def guiMask(self,msg):
        path = os.path.join(self.metadata_path,"mask")
        name = self.proc_frame_id+".jpg"
        self.mask = self.getImg(msg)
        self.img_writer(self.mask,self.proc_frame_id+"_mask")

    def pulse(self,msg):
        pass
