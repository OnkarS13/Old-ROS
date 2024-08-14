#!/usr/bin/env python3
"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""

import sys, os
import rospy
import cv2
import time
import numpy as np

import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()


tf.get_logger().setLevel('ERROR')

from tensorflow.keras.models import load_model

import segmentation_models as sm



class SegmentationModel(object):
    """
    Combined model used to return mask and defect masks of the commodity
    model path and its shape is directly taken from parameters.yaml
    """
    def __init__(self,model_path, in_use):
        #read the related settings from parameter.yaml using rosparam
        # user = os.environ['HOME'].split('/')[2]
        self.model_path = model_path 
        self.model_loaded = in_use

        print((self.model_path+"\n")*8)
        self.loss_func = sm.losses.bce_jaccard_loss

        self.session = tf.Session()
        self.graph = tf.get_default_graph()

        #shapes and crap

        self.img_h = None
        self.img_w = None
        
        '''
        parameters.yaml has a single value for the input shapes as the shapes are symmetrical in nature
        the model must have single input in the fashion of (x,x) or (128,128)
        '''
        self.seg_x  =  int(rospy.get_param('/sortobot/models/shape/segmentation'))
        
        self.loadModel()

    def iou_score(self, gt, pr, class_weights=1., smooth=1, per_image=True, threshold=None):

        if per_image:
            axes = [1, 2]
        else:
            axes = [0, 1, 2]
            
        if threshold is not None:
            pr = tf.greater(pr, threshold)
            pr = tf.cast(pr, dtype=tf.float32)

        intersection = tf.reduce_sum(gt * pr, axis=axes)
        union = tf.reduce_sum(gt + pr, axis=axes) - intersection
        iou = (intersection + smooth) / (union + smooth)

        # mean per image
        if per_image:
            iou = tf.reduce_mean(iou, axis=0)

        # weighted mean per class
        iou = tf.reduce_mean(iou * class_weights)

        return iou

    def unload_model(self):
        tf.keras.backend.clear_session()
        del self.model

    def loadModel(self):
        with self.graph.as_default():
            with self.session.as_default():
                self.model = load_model(self.model_path, custom_objects={"iou_score":self.iou_score,"binary_crossentropy_plus_jaccard_loss":self.loss_func})
               
    
    def dummyOutput(self):
        ''' 
        just pass an empty numpy array from the model. Usually the first frame prediction
        takes insane amount of time. Hence, just get a predition after loading the model and
        and be done with the initial jitter.
        '''
        eg_img = np.zeros(shape=(1000,1000,3),dtype=np.uint8)

        # eg_img = self.preProcessing(eg_img)
        output = self.infer(eg_img)
        print("Shape for dummpy output: ",output.shape)

    def preProcessing(self,img):
        #preprocessing may just require reshaping
        
        return  cv2.resize(img,(self.seg_x,self.seg_x)).reshape([1,self.seg_x,self.seg_x,3]).astype(np.float32)
   
    def postProcessing(self):
    
        #from the output, create a numpy array which will contain the final segmentation masks along axis=0
        collective_mask = np.zeros( shape = (len(self.output),self.seg_x,self.seg_x))

        for i in range(len(self.output)):
            mask = np.argmax(self.output[i], axis=3)
            mask = mask.reshape(self.seg_x,self.seg_x)
            mask = np.uint8(mask*255)
            # mask = cv2.resize(mask, (self.x,self.y))
            collective_mask[i,:,:] = mask
        
        return collective_mask




    def infer(self,img):

        self.y,self.x,_ = img.shape
        
        with self.session.as_default():    
            with self.graph.as_default():
                Input = self.preProcessing(img)
                self.output  = self.model.predict([Input]) 
                

                inference = self.postProcessing()
                
                return inference

