"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""

import cv2
import numpy as np
import scipy.spatial.distance as dist  
import logging

from tensorflow.python.ops import data_flow_ops 
from multilane_sorter.utils.dateTimeLocUtils import DateTimeLocation
import rospy
dtm = DateTimeLocation()

class Measurement(object):

    def __init__(self, *args, **kwargs):
        self.minArea4Contour = None
        self.y_min = int(rospy.get_param("/sortobot/FOV_y_min"))
        self.y_max = int(rospy.get_param("/sortobot/FOV_y_max"))
        self.minimumContourAreaThreshold = float(rospy.get_param("/sortobot/minimumContourAreaThreshold"))


    def orderPoints(self,pts):
            
        xSorted=pts[np.argsort (pts[:, 0]), :]

        leftMost=xSorted[:2, :]
        rightMost=xSorted[2:, :]

        leftMost=leftMost[np.argsort (leftMost[:, 1]), :]
        (tl, bl)=leftMost

        D=dist.cdist (tl[np.newaxis], rightMost, "euclidean")[0]
        (br, tr)=rightMost[np.argsort (D)[::-1], :]

        return np.array ([tl, tr, br, bl], dtype="float32")

    def cleaner(self,mask):
        """
        This function returns a cleaner masks removing all the unnecessary contours 
        """
        img = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (5,5))
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, (3,3))
        
        if self.minArea4Contour is None:
            area = mask.shape[0]*mask.shape[1]
            self.minArea4Contour = area*self.minimumContourAreaThreshold

        return img
    
    def getContour(self,mask):
        """
        This function returns a contour list 
        which are well within the minimum contour area threshold
        """
        thresh=cv2.threshold(mask.astype(np.uint8), 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        contours=cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        
        cnts = [i for i in contours if cv2.contourArea(i)>self.minArea4Contour]
        sort_cnt =  sorted(cnts, key=lambda x: cv2.contourArea(x),reverse=True)
        return sort_cnt

    def getMoments(self, cnt):
        if len(cnt)>5:
            (X, Y), (MA, ma), angle = cv2.fitEllipse(cnt)
        else:
            (x,y,w,h) = cv2.boundingRect(cnt)
            MA,ma = max(w,h), min(w,h)
        M = cv2.moments(cnt)
        cY = int(M["m01"] / (M["m00"] + 0.000001))
        if cY in range(self.y_min,self.y_max):
            centeral_cnt = cnt
            data = (MA,ma)
            return cnt, data
        else:
            return None, None

    def getCorrectContour(self,cnt_ls, double_flag=False):
        """
        This function returns contour pixel diam and the contour 
        """
        if double_flag==False:
            cnt, data = self.getMoments(cnt_ls[0])
            return cnt, data
        else:
            data_ls = []
            for c in cnt_ls:
                cnt,data = self.getMoments(c)
                data_ls.append(data)
            return None, data_ls

    def getCircularityScore(self, contour_lst, minsize):
        circular_lst = []
        for c in contour_lst:
            if cv2.contourArea(c)>minsize:
                periemeter = cv2.arcLength(c, True)
                area = cv2.contourArea(c)
                if periemeter is not None:
                    circularity = 4*3.14*(area/(periemeter*periemeter))
                    circular_lst.append(circularity)
        return circular_lst


    def doubleJoint(self, mask):
        print("jointDoubleDetected")
        dist = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
        _, dist1 = cv2.threshold(dist, 0.5*dist.max(), 255, 0)
        
        # markers = np.zeros(dist.shape, dtype=np.int32)
        dist_8u = dist1.astype('uint8')
        contours, _ = cv2.findContours(dist_8u, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        _,data_ls=self.getCorrectContour(contours,double_flag=True)

        return 3,data_ls,None
        # data_ls = []
        # for c in contours:
        #     _, data = self.getMoments(c)
        #     data_ls.append(data)
            
        # return data_ls
                       
            

    def anomalyDetector(self, mask, minsize=2000, circularity_thresh=0.6):
        """This function is for detecting multiple onions anomaly
        input: 1. list of contours, 2. minimum object size, 3. circularity threshold (0.1 to 0.9).
        returns: True - if anomaly is detected.
                False - if no anomaly is detected."""


        cont_lst = []
        anomaly = 0
        mask = self.cleaner(mask[:,:,0])

        self.contour_list = self.getContour(mask)
        c_list = [cont for cont in self.contour_list if cv2.contourArea(cont)>minsize]  #check only for contours greater than minimum size
        

        self.circular_lst = self.getCircularityScore(c_list, minsize)

        for i in range(len(self.circular_lst)):
            if self.circular_lst[i]>circularity_thresh:
                print("circlarity score :", self.circular_lst[i])
                cont_lst.append(c_list[i])
            
            elif self.circular_lst[i]<circularity_thresh:
                anomaly+=1
                
        if len(cont_lst)>1:
            #double seperated anomaly
            self.main_cnt,self.data_ls = self.getCorrectContour(self.contour_list, double_flag=True)
            return 2, self.data_ls, None

        if anomaly>0:
            #single contour joint double anomaly
            return self.doubleJoint(mask)
            # if len(self.data_ls)==1:
            # elif len(self.data_ls)>1:
            #     return 2, self.data_ls, None
        else:
            return self.measure()
                
    def measure(self):
        """
        This function on input of 3D mask returns 1--> if main contour is present else 0 , 
        bounding rectangle on contour, pixel count of diameter and the main contour
        """
        
        if len(self.contour_list):
            main_cnt,data = self.getCorrectContour(self.contour_list)
            if main_cnt is not None:
                # self.crop_info = cv2.boundingRect(main_cnt)
                return 1, [data],main_cnt
            else:
                return 0 ,[0.00,0.00],None
        else:
            return 0 ,[0.00,0.00],None
