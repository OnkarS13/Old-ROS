#!/usr/bin/env python3

import message_filters # --> for receive synchrnous data from multiple topics, when we want to process them together
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
import time

CRED    = '\033[91m'
CGRN    = '\033[92m'
CBLNK   = '\33[5m'
CEND    = '\033[0m'
CREDBG  = '\33[41m'

class PreProcessing():
    def __init__(self):
        rospy.init_node('preprocessing_node')
        # rospy.loginfo('preprocessing_node started')
        self.lane = rospy.get_namespace().rstrip('/').split('/')[-1] # --> Retrieves the namespace to identify which lane the node is operating on eg. lane_1, lane_2
        self.camera_id_1 = rospy.get_param("~camera_id_1") # --> Gets the parameter ~camera_id_1 from the ROS parameter server. "camera_11" from onion.launch
        self.camera_id_2 = rospy.get_param("~camera_id_2") 
        self.multiplier = rospy.get_param('/sortobot/multiplier/' + self.lane) # --> Retrieves the multiplier parameter specific to the lane from the ROS parameter server
        self.current_season = rospy.get_param('/sortobot/models/in_use')
        self.bridge = CvBridge() # -->  Initializes a CvBridge object, which is used to convert between ROS image messages and OpenCV images. 
        self.output = inference() # --> Initializes an instance of the custom message type inference, which will hold the output of the preprocessing.
        
        #subscribers
        self.act_image = message_filters.Subscriber("actuator/image_raw", Image) # --> Subscribes to the actuator/image_raw topic, which publishes images of type sensor_msgs/Image.
        self.non_act_image = message_filters.Subscriber("non_actuator/image_raw", Image) # --> Subscribes to the non_actuator/image_raw topic, which also publishes images of type sensor_msgs/Image.
        ts = message_filters.ApproximateTimeSynchronizer([self.act_image, self.non_act_image], 1, 0.09) # --> Creates an ApproximateTimeSynchronizer object that synchronizes messages from the two image topics (actuator/image_raw and non_actuator/image_raw). The messages are synchronized with a queue size of 1 and a time difference tolerance of 0.09 seconds.
        ts.registerCallback(self.image_callback) # --> Registers the image_callback method as the callback function to be executed when synchronized messages are received.

        #publishers
        self.ai_pub = rospy.Publisher('ai_inference_channel', inference, queue_size=1) # --> Creates a publisher for the ai_inference_channel topic, which will publish messages of type inference.
        self.mask_pub1 = rospy.Publisher('preprocessing_act',Image, queue_size=1) # --> Creates a publisher for the preprocessing_act topic, which will publish processed images of type sensor_msgs/Image for the actuator camera.
        self.mask_pub2 = rospy.Publisher('preprocessing_non_act',Image,queue_size=1)

        self.model = Segmentation_model(model_path = "/home/agrograde/agrograde_ws/src/multilane_sorter/ai_models/30th_july_2024/four_class_onnx_model.onnx") # --> Initializes an instance of the Segmentation_model class, passing the path to the ONNX model file. This model will be used for image segmentation tasks.
    
    
    # --> To process synchronized images from two cameras, perform decision-making, and save the images for further analysis.
    def image_callback(self, img1, img2): # --> This method takes two arguments, img1 and img2, which are the synchronized image messages from the actuator and non-actuator cameras.
        self.output.header.stamp = rospy.Time.now() # --> Sets the timestamp of the output message to the current ROS time. This is essential for keeping track of when the data was processed.
        
        rgb_time = time.time()
        img_array1 = self.bridge.imgmsg_to_cv2(img1, desired_encoding = "rgb8") # --> Converts the first ROS image message (img1) to an OpenCV image array with the BGR8 encoding using CvBridge.
        img_array2 = self.bridge.imgmsg_to_cv2(img2, desired_encoding = "rgb8") #rgb8 ## bgr8 #rgb8## bgr8 #### har time rgb on ros
              
        self.decision(img_array1, img_array2) # --> Calls the decision method, passing the two OpenCV image arrays. This method is expected to perform some processing and decision-making based on the images.
        
        # saving image path
        path1 = "/home/agrograde/agrograde_ws/src/multilane_sorter/assets/images/{0}/{1}/".format(self.lane, self.camera_id_1)   # 4 lines for image saving # --> Constructs the directory path where the first image will be saved. The path includes the lane identifier and the camera ID.
        path2 = "/home/agrograde/agrograde_ws/src/multilane_sorter/assets/images/{0}/{1}/".format(self.lane, self.camera_id_2)

        io.imsave(path1+str(img1.header.seq)+"_"+".jpg",img_array1) # --> Saves the first image to the specified path. The filename includes the image sequence number. 
        io.imsave(path2+str(img2.header.seq)+"_"+".jpg",img_array2)
        

    # --> This function prepares and publishes a ROS message containing defect detection results and size information for further processing or analysis in the ROS network.
    def message(self, array_1):

        array = [round(item, 2) for item in array_1] # --> The function iterates through the array_1 list and rounds each value to two decimal places.
    
        array[5] = self.multiplier * array[5] # --> The sixth element (index 5) in the array, representing the size, is multiplied by a predefined multiplier. This could be for calibration or scaling purposes.

        # --> The rounded values from the array are assigned to the corresponding fields in the self.output message. This message likely represents the inference results from some image processing or defect detection algorithm:
        self.output.sprout = array[0]
        self.output.peeled = array[1]
        self.output.rotten = array[2]
        self.output.blacksmut = array[3]
        self.output.double = array[4]
        self.output.size = array[5]   # hard coded size for testing

        self.ai_pub.publish(self.output) # --> The updated self.output message is published on a ROS topic (ai_inference_channel). This makes the processed data available to other nodes in the ROS ecosystem.
        rospy.loginfo(self.output) # --> The output message is logged for debugging or informational purposes
       

    # --> To process two input images, perform inference using a segmentation model, and publish the inference results.    
    def decision(self, img_array1, img_array2): # --> img_array1 and img_array2, which are the OpenCV image arrays from the actuator and non-actuator cameras, respectively.
        
        t = time.time() # --> Stores the current time in t to measure the time taken for the processing steps.
        array = self.model.get2_img(img_array1, img_array2) # --> Calls the get2_img method of the Segmentation_model class, passing the two image arrays. This method performs inference using the segmentation model and returns an array of results.
        print("the array = ", array)
        print(f"time taken for processing = {time.time() - t}") # --> Prints the time taken for processing the images and performing inference. This is calculated by subtracting the initial time t from the current time.
        self.message(array) # --> Calls the message method, passing the array of inference results. This method is responsible for packaging and publishing the inference results.
        

class Segmentation_model():
    def __init__(self, model_path): 
        
        self.model_path = model_path # --> This line assigns the provided model_path to the instance variable self.model_path. The model_path is the file path to the ONNX model that will be used for inference.
    
        self.session = onnxruntime.InferenceSession(self.model_path) # --> This line initializes an ONNX runtime inference session using the specified model path. The onnxruntime.InferenceSession class loads the ONNX model and prepares it for running predictions.
        self.input_name = self.session.get_inputs()[0].name # --> This line retrieves the name of the input node of the ONNX model. The get_inputs() method returns a list of input nodes, and [0] accesses the first input node. .name extracts the name of this input node. This name is needed to provide input data to the model during inference.
        self.output_names = [output.name for output in self.session.get_outputs()] # --> This line retrieves the names of the output nodes of the ONNX model. The get_outputs() method returns a list of output nodes, and the list comprehension [output.name for output in self.session.get_outputs()] extracts the names of these output nodes. These names are needed to retrieve the output data from the model after inference.

    def predict(self, image):
        
        result = self.session.run(self.output_names, {self.input_name: image})
        
        return result
    
    
    # --> This function calculates the percentage area of a specific region within an entire area.
    def getPercentArea(self, full_mask, region_mask):

        total_area = np.dot(full_mask.flatten(), np.ones_like(full_mask.flatten())) # --> 1.	Flatten full_mask: Convert the 2D array full_mask into a 1D array. 2.	Create a 1D array of ones: The same length as the flattened full_mask. 3.	Dot Product: The dot product of the flattened full_mask and the array of ones sums all the values in full_mask. This gives the total area covered by full_mask.
        region_area = np.dot(region_mask.flatten(), np.ones_like(region_mask.flatten())) # --> 1.	Flatten region_mask: Convert the 2D array region_mask into a 1D array. 2.	Create a 1D array of ones: The same length as the flattened region_mask. 3.	Dot Product: The dot product of the flattened region_mask and the array of ones sums all the values in region_mask. This gives the area covered by region_mask.

        area_percentage = (region_area/total_area) * 100 # --> 1.	Divide region_area by total_area: This gives the proportion of the region area within the total area. 2.	Multiply by 100: Converts the proportion to a percentage.

        return area_percentage
    

    # --> To combine defect information and size from two input images and return the aggregated results.
    def get2_img(self, img_array1, img_array2):

        # l1 & l2 are the lists of the same image array of the defect percent Area (actuator side and no-actuator side) also for s1 & s2

        defects = [] # --> Initializes an empty list to store the defect information.
        s1, s2 = 0,0 # --> Initializes variables to store the sizes of the objects detected in the two images.
        
        # l1 has the defects list and s1 has the size
        l1, s1 = self.getPrediction_values(img_array1) # --> Calls the getPrediction_values method for the first image array. This method returns a list of defects (l1) and the size (s1).
        l2, s2 = self.getPrediction_values(img_array2)

        # Defect percent Area values in the list
        # l1 = [sprout, peeled, rotten, black_smut, double] --> Actuator side
        # l2 = [sprout, peeled, rotten, black_smut, double] --> Non-Actuator side
        # s1 = size of contour of image array --> Actuator side
        # s2 = size of contour of image array --> Non-Actuator side 

        defects = [max(l1[0], l2[0]), max(l1[1], l2[1]), max(l1[2], l2[2]), max(l1[3], l2[3]), max(5, 10)] # --> Combines the defect information from both images. For each type of defect, it takes the maximum value between the two images. This ensures that the detected defect with the highest severity is considered.

        size = max(s1, s2) # --> Determines the size of the object by taking the maximum value between the sizes detected in the two images. This ensures that the largest detected size is considered.
        
        defects.append(size) # -->  Appends the determined size to the defects list.

        # Defects list is made with maximum values from l1 & l2 lists and maximum size from both s1 & s2 values is appended for Single image array
        # defects = [sprout, peeled, rotten, black_smut, double, size]
 
        print(f"defects  - {defects}") 
        
        return defects # This defects list is returned which is for a single onion.
        
        
    # --> To predict defect areas and size from an input image using a segmentation model and return the calculated percentages and size.
    def getPrediction_values(self, img_array): 
        
        h,w = 224,224 # --> Sets the height and width to 224 pixels.
        im = cv2.resize(img_array,(h,w)) # --> Resizes the image to 224x224 pixels.
        I = im.astype(np.float32) # --> Converts the image to float32 data type.
        I = I.reshape([1, h, w, 3]) # --> Reshapes the image to the required input shape for the model (batch size, height, width, channels).
        start = time.time()
        
        
        preds = self.predict(I) # --> Performs the prediction using the predict method of the segmentation model.
        print(f"Inference time: {(time.time() - start):.2f} sec") # --> Prints the time taken for the inference.
        
        # Post processing - Argmax [p(absence), p(presence)] --> [0, 1] indices
        sp = np.argmax(preds[0], axis = 3) # --> Extracts the sprout segmentation mask, it accesses the first array in the preds which is sprout --> [batch_size, 224, 224, 2] and accesses axis = 3 means last dimension, 
        sp = sp.reshape([h,w]) # which has two probabilities of the absence and presence of that class at that pixel, from that index of the higher probability is areturned with argmax.

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

        im = cv2.cvtColor(im , cv2.COLOR_BGR2RGB) # --> This line converts the image from BGR color space to RGB color space using OpenCV. By default, OpenCV reads images in BGR format, so this conversion is often necessary for consistency with other libraries that use RGB.
        all_masks = [sp,pl,ro,bs,bg]
        # all_masks = [sp,pl,ro,bs,db,bg]

        # Size
        gray_img = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY) # --> This line converts the RGB image to a grayscale image. Grayscale images are often used for operations like thresholding and contour detection because they simplify the image data.
        ret, binary = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU) # --> This line applies binary thresholding to the grayscale image. cv2.threshold converts the grayscale image into a binary image (black and white) based on a threshold value. cv2.THRESH_OTSU is used to automatically determine the optimal threshold value using Otsu’s method.
        contours, hierarchy = cv2.findContours(binary, mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_NONE) # --> This line finds contours in the binary image. Contours are curves that join all the continuous points along a boundary with the same color or intensity. cv2.RETR_TREE retrieves all the contours and reconstructs a full hierarchy of nested contours. cv2.CHAIN_APPROX_NONE stores all the contour points.
        
        # --> This loop iterates through all the contours and calculates the area of each contour using cv2.contourArea. It keeps track of the largest contour found by comparing each contour’s area with max_area. The largest contour is stored in biggest_contour.
        max_area = 0
        biggest_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                biggest_contour = contour
        
        ellipse = cv2.fitEllipse(biggest_contour) # --> This line fits an ellipse to the largest contour found. cv2.fitEllipse returns the parameters of the fitted ellipse, which can be useful for measuring the size and orientation of the contour.
        size = max(ellipse[1]) * 1.47489 # --> This line calculates the size of the onion based on the major axis of the fitted ellipse (ellipse[1]). The size is scaled by a factor of 1.47489. This scaling factor could be an empirical value determined based on specific requirements or calibration. The calculated size is then printed.
        print("size of the onion {}".format(size))
      
        
        # we were using binary 1 now we have changed to gray
        sprout_area = self.getPercentArea(bg, sp)#we were using binary 1 now we have changed to gray

        peeled_area = self.getPercentArea(bg, pl)#we were using binary 1 now we have changed to gray

        rotten_area = self.getPercentArea(bg, ro)#we were using binary 1 now we have changed to gray

        black_smut_area = self.getPercentArea(bg, bs)#we were using binary 1 now we have changed to gray

        # double_area = 10 #we were using binary 1 now we have changed to gray

        background_area = self.getPercentArea(bg, bg) #we were using binary 1 now we have changed to gray

        total_area = background_area

        r1,r2,r3,r4 = ((sprout_area*100)/total_area), ((peeled_area*100)/total_area), ((rotten_area*100)/total_area), ((black_smut_area*100)/total_area),
    
        final_percentage_features = [r1,r2,r3,r4]

        # For 5 classes
        # r1,r2,r3,r4,r5=((sprout_area*100)/total_area), ((peeled_area*100)/total_area), ((rotten_area*100)/total_area), ((black_smut_area*100)/total_area), ((double_area*100)/total_area)
    
        # final_percentage_features = [r1,r2,r3,r4,r5]

        print(final_percentage_features,size)
        print("size of onion:", size)
        return final_percentage_features, size


     

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
        




