"""
Author: Nikhil Pandey, Vasan Naddoni
copyright @ Occipital Technologies Pvt Ltd
"""
import os, sys
sys.path.insert(0, os.path.abspath('..'))
from pypylon import pylon
from multiprocessing import Process
import cv2




class BaslerCam:
    
    """
    Custom class to use basler software with python support
    1. Starts camera initialization
    2. For now can only set exposure

    """
    def __init__ (self,serialnumber,first_cam,pulse_capture, BaslerFlag=0,exposure=None):
        info = pylon.DeviceInfo()
        info.SetSerialNumber(serialnumber)
        if first_cam:
            self.camera=pylon.InstantCamera (pylon.TlFactory.GetInstance().CreateFirstDevice(info))
        else :
            self.camera=pylon.InstantCamera (pylon.TlFactory.GetInstance().CreateDevice(info))

        self.camera.Open()
        if exposure is None:
            exposure = 1400
        self.camera.ExposureTime.SetValue(int(exposure))

        if BaslerFlag ==1:
            self.camera.TriggerSelector.SetValue("FrameBurstStart")
            self.camera.TriggerSource.SetValue("Line1")
            self.camera.TriggerMode.SetValue("On")
            if pulse_capture == "FE":
                self.camera.TriggerActivation.SetValue('FallingEdge')
            if pulse_capture == "RE":
                self.camera.TriggerActivation.SetValue('RisingEdge')


        # Grabing continuously (video) with minimal delay
        self.camera.StartGrabbing (pylon.GrabStrategy_LatestImageOnly)
        self.converter=pylon.ImageFormatConverter ()

        # converting to opencv bgr format
        self.converter.OutputPixelFormat=pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment=pylon.OutputBitAlignment_MsbAligned
        

    def read (self):
        """
        Starts returning frames 
        camera time out set to 10 seconds
        """
        grabResult=self.camera.RetrieveResult (10000, pylon.TimeoutHandling_ThrowException)

        if grabResult.GrabSucceeded():
            # Access the image data
            self.image=self.converter.Convert(grabResult)
            self.img=self.image.GetArray ()
            return True,self.img
        
        else:
            return  False, None

    def release (self):
        self.camera.StopGrabbing ()
        del self.camera
