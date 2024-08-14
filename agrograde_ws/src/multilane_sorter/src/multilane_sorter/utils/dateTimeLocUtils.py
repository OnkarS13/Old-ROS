"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""

import sys, os
sys.path.insert(0, os.path.abspath('..'))
from datetime import datetime,date
import pytz


class DateTimeLocation(object):

    def __init__(self,defalut_storage=None):

        if defalut_storage is None:
            self.defalutStorage="./storage"
        else:
            self.defalutStorage = defalut_storage


    def prepareStorage(self,folderName):

        newPath = os.path.join(self.defalutStorage,folderName)
        print(newPath)
        if not os.path.exists(newPath):
            os.mkdir(newPath)
        return newPath

    def getTimeForDB(self):
        tz = pytz.timezone('Asia/Kolkata')
        now = datetime.now().replace(tzinfo = tz)
        return now

    def getTime(self):

        tz = pytz.timezone('Asia/Kolkata')
        now = datetime.now().replace(tzinfo = tz)
        return  now.strftime('%H:%M:%S')


    def getDate(self):
        return date.today().strftime("%b-%d-%Y")

    def getLocation(self,name=None,suffix = None):

        if suffix is None:
            suffix = ".mp4"
            
        folderName  = str(self.getDate())

        if name is None:
            fileName    = str(self.getTime())+suffix
        
        else:
            fileName = name
        
        newPath = self.prepareStorage(folderName)
        # print(newPath)
        return os.path.join(newPath,fileName),newPath