#!/usr/bin/env python3

"""
Author: Vasan Naddoni
copyright @ Occipital Technologies Pvt Ltd
"""

import sys, os
sys.path.insert(0, os.path.abspath('..'))
import cv2
import time
import numpy as np

try:
    import tensorflow.compat.v1 as tf
    tf.disable_v2_behavior()
except:
    import tensorflow as tf

from utils.computerVision import Measurement
tf.get_logger().setLevel('ERROR')
from utils.dateTimeLocUtils import DateTimeLocation
dtm = DateTimeLocation()
import logging
import keras
from tensorflow.keras.models import load_model
from keras import backend as K

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)

class CombinedModel(object):
    def __init__(self,seg_input_shape,def_input_shape,col_input_shape,model_name):
        self.segmentation_input_shape = seg_input_shape 
        self.defect_input_shape = def_input_shape
        self.color_input_shape = col_input_shape
        self.focused_img = None
        self.defect_img = None

        self.session = tf.Session()
        self.graph = tf.get_default_graph()



        with self.graph.as_default():
            with self.session.as_default():
                self.model = load_model(model_name)



    def preprocess(self,img):
    
        x1,y1 = self.segmentation_input_shape
        x2,y2 = self.defect_input_shape
        x3,y3 = self.color_input_shape

        img_1 = cv2.resize(img,self.segmentation_input_shape).reshape([1,x1,y1,3]).astype(np.float32)
        
        if self.focused_img is not None:
            self.focused_img = cv2.cvtColor(self.focused_img,cv2.COLOR_RGB2BGR)
            img_2 = cv2.resize(self.defect_img,self.defect_input_shape).reshape([1,x2,y2,3]).astype(np.float32)
            img_3 = cv2.resize(self.focused_img,self.color_input_shape).reshape([1,x3,y3,3]).astype(np.float32)

        else:
            img_2 = cv2.resize(img,self.defect_input_shape).reshape([1,x2,y2,3]).astype(np.float32)
            img_3 = cv2.resize(img,self.color_input_shape).reshape([1,x3,y3,3]).astype(np.float32)

        return img_1,img_3
       
    def postprocessing(self):

        segment_map         = np.argmax(self.output[0],axis=3) 
        self.segment_mask   = 255*segment_map.reshape([224,224])
        self.segment_mask   = self.segment_mask.astype("uint8")
        self.segmented_mask = cv2.resize(self.segment_mask,(self.w,self.h))


    def postprocessing_defects(self):
        
        defect_candidates = []
        
        self.def_seg = np.zeros(shape=(224,224,6),dtype=np.uint8)

        # def_13 = np.array()

        for i in range(6):
            
            mask = np.argmax(self.output[i+1], axis=3)
            mask = mask.reshape(self.defect_input_shape)
            mask = np.uint8(mask*255)
            mask = cv2.resize(mask, (224,224))
            self.def_seg[:,:,i] = mask
            extent = np.sum(mask)

            defect_candidates.append(extent)

        defects = np.squeeze(np.array(defect_candidates)/(sum(defect_candidates)+1))*100

        self.defect_array = np.array([defects[0],defects[1],defects[2],defects[3],defects[4],defects[5]]).round(2).tolist()

        self.defect_images =  np.hstack((self.def_seg[:,:,:3],self.def_seg[:,:,3:]))

        self.defect = 0


    def postprocessing_colors(self):
        self.color  = np.argmax(self.output[6])

    def predict_quantities(self,img):
       
        with self.session.as_default():
            
            with self.graph.as_default():
                
                self.img = img
                self.h,self.w,_ = self.img.shape
                I1,I3 = self.preprocess(img)


                self.output = self.model.predict([I1,I3]) 
                
                # print("*"*10)
                # print(self.output[0].shape)
                # print(self.output[1].shape)
                # print(self.output[2].shape)
                # print(self.output[3].shape)
                # print(self.output[4].shape)
                # print(self.output[5].shape)
                # print(self.output[6].shape)

                self.defect_img = img
            
                self.postprocessing()
            
                if self.focused_img is not None:

                    self.postprocessing_defects()
                    self.postprocessing_colors()


