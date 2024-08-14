#boto3==1.10.41 
#rq==1.1.0
#sudo apt-get install redis-server
#setup creditails of aws


import sys, os
sys.path.insert(0, os.path.abspath('..'))
from utils.dateTimeLocUtils import DateTimeLocation
dtm = DateTimeLocation()
import rospy
import time
import pymongo
from std_msgs.msg import String

import boto3
# dynamo = boto3.resource('dynamodb')
# cloud_table = dynamo.Table('machine_data')

error_channel = rospy.Publisher('/'+os.environ['HOME'].split('/')[2]+'/error_channel', String,queue_size=1)


class database_arch(object):
    def __init__(self):
        try:
            myclient = pymongo.MongoClient("mongodb://localhost:27017/")
            mydb = myclient[rospy.get_param('sortobot/database_config/db_name')]
            self.mycol = mydb[rospy.get_param('sortobot/database_config/db_table')]

            sumdb = myclient[rospy.get_param('sortobot/database_config/summary_db_name')]
            self.sumcol = sumdb[rospy.get_param('sortobot/database_config/summary_table')]

            cloud_table = 'machine_data'
            self.session = boto3.Session(
                aws_access_key_id="AKIAY4C2ZLSG2X37N75I",
                aws_secret_access_key="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",
                region_name="ap-south-1"
            )

            self.dynamo = self.session.resource('dynamodb')
            self.table = self.dynamo.Table(cloud_table)
                    
        except Exception as err:
            error_channel.publish(str(err))

    def store_data(self,Item):

        try:
            if rospy.get_param('sortobot/gui/status'):
                # self.table.put_item(Item = Item)
                # print(Item) 
                pass
        except Exception as err:
            self.mycol.insert_one(Item)
            print(err)
        return 0

    def store_data_summary(self,summary_dict):

        self.sumcol.insert_one(summary_dict)
        return 0


    def get_data(self,fid,date):

        data = []

        myquery = { "Date": date, "Fid": fid }
        mydoc = self.mycol.find(myquery)
        for x in mydoc:
            data.append(x)

        return data

    def get_data_summary(self,fid):

        data = []

        myquery = {"orderID": fid }
        mydoc = self.sumcol.find(myquery)
        for x in mydoc:
            data.append(x)

        return data

    # x = get_data(1,"Dec-20-2019")
    # print(x)