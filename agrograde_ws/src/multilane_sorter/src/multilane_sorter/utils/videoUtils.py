"""
Author: Nikhil Pandey
copyrigh @ Occipital Technologies Pvt Ltd
"""
import sys, os
sys.path.insert(0, os.path.abspath('..'))
from vidgear.gears import WriteGear
import cv2
from datetime import datetime,date
import pytz
from utils.dateTimeLocUtils import DateTimeLocation

dtl = DateTimeLocation(defalut_storage="../storage")

class VideoWriter(object):

    def __init__(self, fps=None,location=None,name=None,path=None):
        """
        fps     : Set FPS for playback
        location: Comself.locationplete path of file
        name    : Name of file; provide when not using location
        path    : Path of the folder 
        """
        self.fps = fps
        self.location = dtl.getLocation(suffix=".mp4")
                           
        if  self.fps is None:
            self.fps = 30
        

        output_params = {"-vcodec":"libx264", "-crf": 0, 
                            "-preset": "fast", "-fps": self.fps}
                            
        self.writer = WriteGear(output_filename = self.location,
                            compression_mode=False,
                           THREADED_QUEUE_MODE=True,logging = False, **output_params) 


    def writeFrame(self,frame):
        self.writer.write(frame)

    def closeWriter(self):
        self.writer.close()


