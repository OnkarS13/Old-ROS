# from google.cloud import storage
import sys, os
sys.path.insert(0, os.path.abspath('..'))

import os
import datetime
# import wget
# import boto3/
# from botocore.exceptions import NoCredentialsError
# import urllib
# from urllib.request import urlopen
# from tqdm import tqdm
import glob
# import requests,json
# import cv2,base64

import concurrent.futures as futures
import getpass
# from utils.computerVision import Measurement

import requests,json


class CloudUploader():
	def __init__(self):
		self.user = getpass.getuser()
		self.curr_image_path_list = []
		self.absolutePath = "/home/"+self.user+"/"+str(datetime.datetime.now().year)
		self.payload = {
		"type": "TOKEN",
		"authorizationToken": "incoming-client-token",
		"methodArn": "arn:aws:execute-api:ap-south-1:123456789012:example/prod/POST/{proxy+}",
		}

		self.headers = {'content-type': 'image/jpeg'}
		self.server = "https://yrjk6scw89.execute-api.ap-south-1.amazonaws.com/initial/imageUpload"		

	def getUrls(self,folder_path, img_name):

		#get presigned urls
		response = requests.post(self.server, data= json.dumps(self.payload), headers=self.headers)
		data = response.json()
		
		url = data['PreSignedUrl']

		#upload data using the urls
		with open(os.path.join(folder_path,img_name), 'rb') as f:
			files = {'file': (img_name, f)}
			http_response = requests.put(url,files=files)
			# print("response: ",http_response)
			# self.requestPut(url,files)
			# with futures.ProcessPoolExecutor(max_workers=5) as executor:
			# 	executor.submit(self.requestPut,url,files)


	# def requestPut(self,url,file):

	
	def multiUpload(self,imgpath_lst):
		"""Input - list image paths to upload
		output - response code(print)"""

		for impath in imgpath_lst:

			#get current image name
			img_name = impath.split('/')[-1]

			#get folder path to save on bucket
			path = impath.split('/')
			p = '/'
			folder_path = p.join(path[:-1])
			# print(folder_path)
			#update the payload
			self.payload.update({"selected_file":img_name,"folder_path":folder_path})
			
			#upload the image
			self.getUrls(folder_path, img_name)


	def upload_on_cloud(self):

		self.image_path_list = glob.glob(self.absolutePath+"/*/*/*/*/*.jpg")
		self.latest_images = [i for i in self.image_path_list if i not in self.curr_image_path_list]
		self.multiUpload(self.latest_images)
		self.curr_image_path_list = self.image_path_list


if __name__ == "__main__":
	CloudUploader = CloudUploader()
	CloudUploader.upload_on_cloud()