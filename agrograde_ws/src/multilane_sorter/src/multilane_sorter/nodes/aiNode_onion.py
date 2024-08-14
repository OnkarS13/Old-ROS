#!/usr/bin/env python3

import message_filters
import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String, Int8
from multilane_sorter.msg import inference
# from std_msgs.msg import Time
import time
from skimage import io
import onnx
import onnxruntime

import numpy as np
import cv2
import os
from tqdm import tqdm_notebook as tqdm
import matplotlib.pyplot as plt
import tensorflow as tf
# from tensorflow.keras.models import load_model, Model
# from tensorflow.keras import layers
# from tensorflow.keras.layers import Input,Conv2D,BatchNormalization,UpSampling2D,concatenate
import time

# from keras.layers import *
# from keras.models import *

CRED    = '\033[91m'
CGRN    = '\033[92m'
CBLNK   = '\33[5m'
CEND    = '\033[0m'
CREDBG  = '\33[41m'

class PreProcessing():
    def __init__(self):
        rospy.init_node('preprocessing_node')
        # rospy.loginfo('preprocessing_node started')
        self.lane = rospy.get_namespace().rstrip('/').split('/')[-1]
        self.camera_id_1 = rospy.get_param("~camera_id_1")
        self.camera_id_2 = rospy.get_param("~camera_id_2")
        self.bridge = CvBridge()
        self.output = inference()
        #subscribers
        self.act_image = message_filters.Subscriber("actuator/image_raw", Image)
        self.non_act_image = message_filters.Subscriber("non_actuator/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.act_image,self.non_act_image],1,0.09)
        ts.registerCallback(self.image_callback)
        # rospy.Subscriber("actuator/image_raw", Image, self.image_callback1)
        # rospy.Subscriber("non_actuator/image_raw", Image, self.image_callback2)

        #publishers
        self.ai_pub = rospy.Publisher('ai_inference_channel', inference, queue_size=1)
        self.mask_pub1 = rospy.Publisher('preprocessing_act',Image,queue_size=1)
        self.mask_pub2 = rospy.Publisher('preprocessing_non_act',Image,queue_size=1)

        self.multiplier = rospy.get_param('/sortobot/multiplier/'+self.lane)
        self.current_season = rospy.get_param('/sortobot/models/in_use')

        self.model = Segmentation_model(model_path="//home/agrograde/agrograde_ws/src/multilane_sorter/ai_models/30th_july_2024/four_class_onnx_model.onnx")
        

      
    def image_callback(self,img1,img2):
        self.output.header.stamp = rospy.Time.now()
        
        rgb_time =time.time()
        img_array1 = self.bridge.imgmsg_to_cv2(img1, desired_encoding="bgr8")#rgb8## bgr8 #### har time rgb on ros
        img_array2 = self.bridge.imgmsg_to_cv2(img2, desired_encoding="bgr8")#rgb8 ## bgr8
              
        self.decision(img_array1, img_array2)
        # path1 = "/home/agrograde/potato_ws/src/multilane_sorter/assets/images/{0}/{1}/".format(self.lane, self.camera_id_1)   # 4 lines for image saving
        path1 = "/home/agrograde/agrograde_ws/src/multilane_sorter/assets/images/{0}/{1}/".format(self.lane, self.camera_id_1)   # 4 lines for image saving
        path2 = "/home/agrograde/agrograde_ws/src/multilane_sorter/assets/images/{0}/{1}/".format(self.lane, self.camera_id_2)

        io.imsave(path1+str(img1.header.seq)+"_"+".jpg",img_array1)
        io.imsave(path2+str(img2.header.seq)+"_"+".jpg",img_array2)
        

    def message(self,array_1):

        array = [round(item, 2) for item in array_1]
    
        array[5] = self.multiplier*array[5]
        self.output.sprout = array[0]
        self.output.peeled = array[1]
        self.output.rotten = array[2]
        self.output.blacksmut = array[3]
        self.output.double = array[4]

        self.output.size = array[5]                  #hard coded size for testing
        self.ai_pub.publish(self.output)
        rospy.loginfo(self.output)
       
        # rospy.loginfo("the publishing data from preprocessing node is...")
    def decision(self, img_array1, img_array2):
        # rospy.loginfo(self.model)
        #array = [sprout,black_smut,rotten,size]
        t = time.time()
        array = self.model.get2_img(img_array1,img_array2) 
        print("the array = ", array)
        print(f"time taken for processing = {time.time()-t}") 
        self.message(array)
        

class Segmentation_model():
    def __init__(self,model_path):
        
        self.model_path = model_path
    
        self.session = onnxruntime.InferenceSession(self.model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]

    def predict(self,image):
        
        result = self.session.run(self.output_names, {self.input_name: image})
        
        return result
    
    
    def getPercentArea(self, full_mask, region_mask):

        total_area = np.dot(full_mask.flatten(), np.ones_like(full_mask.flatten()))
        region_area = np.dot(region_mask.flatten(), np.ones_like(region_mask.flatten()))

        area_percentage = (region_area/total_area)*100

        return area_percentage
    

    def get2_img(self,img_path1,img_path2):

        defects = []
        s1,s2 = 0,0
        
        l1,s1=self.getPrediction_values(img_path1)
        l2,s2=self.getPrediction_values(img_path2)
        # bs1 = [l1[1],l2[1]]
        # bs = sum(bs1)/len(bs1)
        
        #defects =  [(x + y) / 2 for x, y in zip(l1, l2)]
        #sp = [max(l1[0],l2[0]),]
        defects = [max(l1[0],l2[0]), max(l1[1],l2[1]), max(l1[2], l2[2]), max(l1[3],l2[3]), max(5, 10)]
        # defects = [max(l1[0],l2[0]), bs, max(l1[2], l2[2])]

        
        size = max(s1,s2)
        # defects.append()
        defects.append(size)
        print(f"defects  - {defects}")
        
        return defects
        
        
    def getPrediction_values(self, img_path):
        h,w = 224,224
        
        #im = cv2.imread(img_path)
        im = cv2.resize(img_path,(h,w))
        I = im.astype(np.float32)
        I = I.reshape([1, h, w, 3])
        start = time.time()
        
        
        preds = self.predict(I)
        print(f"Inference time: {(time.time() - start):.2f} sec")
        
        
        sp = np.argmax(preds[0], axis=3)
        sp = sp.reshape([h,w])

        pl = np.argmax(preds[1], axis = 3)
        pl = pl.reshape([h,w])
       
        ro = np.argmax(preds[2], axis=3)
        ro = ro.reshape([h,w])
        
        bs = np.argmax(preds[3], axis=3)
        bs = bs.reshape([h,w])

        # db = np.argmax(preds[3], axis=3)
        # db = db.reshape([h,w])
        
        bg = np.argmax(preds[4], axis=3)
        bg = bg.reshape([h,w])

        im = cv2.cvtColor(im , cv2.COLOR_BGR2RGB)

       
        all_masks = [sp,pl,ro,bs,bg]
        # all_masks = [sp,pl,ro,bs,db,bg]


        gray_img = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        ret,binary = cv2.threshold(gray_img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        contours,hierarchy = cv2.findContours(binary,mode = cv2.RETR_TREE,method = cv2.CHAIN_APPROX_NONE)
        max_area = 0
        biggest_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                biggest_contour = contour
        
        ellipse = cv2.fitEllipse(biggest_contour)
        size = max(ellipse[1]) * 1.47489
        print("size of the onion {}".format(size))
      
        
      
        

        
        
        

        

#         we were using binary 1 now we have changed to gray
        sprout_area = self.getPercentArea(bg, sp)#we were using binary 1 now we have changed to gray

        peeled_area = self.getPercentArea(bg, pl)#we were using binary 1 now we have changed to gray

        rotten_area = self.getPercentArea(bg, ro)#we were using binary 1 now we have changed to gray

        black_smut_area = self.getPercentArea(bg, bs)#we were using binary 1 now we have changed to gray

        # double_area = 10 #we were using binary 1 now we have changed to gray

        background_area = self.getPercentArea(bg, bg) #we were using binary 1 now we have changed to gray

        total_area = background_area

        r1,r2,r3,r4=((sprout_area*100)/total_area), ((peeled_area*100)/total_area), ((rotten_area*100)/total_area), ((black_smut_area*100)/total_area),
    
        final_percentage_features = [r1,r2,r3,r4]

        # For 5 classes
        # r1,r2,r3,r4,r5=((sprout_area*100)/total_area), ((peeled_area*100)/total_area), ((rotten_area*100)/total_area), ((black_smut_area*100)/total_area), ((double_area*100)/total_area)
    
        # final_percentage_features = [r1,r2,r3,r4,r5]

        print(final_percentage_features,size)
        print("size of onion:",size)
        return final_percentage_features,size


     

if __name__ == '__main__':
    t_ai = time.time()

    gpus = tf.config.experimental.list_physical_devices("GPU")
    if gpus:
        try:
            tf.config.experimental.set_virtual_device_configuration(gpus[0],[tf.config.experimental.VirtualDeviceConfiguration(memory_limit = 1024)])
        except RuntimeError as e:
            print(e)

    node = PreProcessing()
     
     
  
    rospy.spin() 
    print(f"time taken for Ai_node = {time.time()-t_ai}")
    print("main bhai type hoon",type(node))
        










