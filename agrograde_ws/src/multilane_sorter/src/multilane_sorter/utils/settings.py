"""
Author: Nikhil Pandey
copyright @ Occipital Technologies Pvt Ltd
"""
import os,sys
sys.path.insert(0, os.path.abspath('..'))
import rospy



class CommoditySettings(object):
    def __init__(self,commodity):
        user = os.environ['HOME'].split('/')[2]
        name = str(user.split('_')[1])
        self.path = os.path.join(rospy.get_param('sortobot/config/relative_path_'+name),"assets")
        
        print(self.path)
        self.combined_model_name = rospy.get_param('sortobot/models/'+commodity+'/combined/name')


        self.bin_label = ['Small','Medium','Large']

        seg_x = rospy.get_param('sortobot/models/'+commodity+'/shape/segmentation')
        def_x = rospy.get_param('sortobot/models/'+commodity+'/shape/defect')
        col_x = rospy.get_param('sortobot/models/'+commodity+'/shape/color')

        self.segmemtation_model_shape = (seg_x,seg_x)
        self.defect_model_shape  = (def_x,def_x)
        self.color_model_shape  = (col_x,col_x)

        self.color_dict = dict((v,k) for k,v in rospy.get_param('sortobot/config/settings/'+commodity+'/color_dict').items())

        self.defect_dict = dict((v,k) for k,v in rospy.get_param('sortobot/config/settings/'+commodity+'/defect_dict').items())

        self.combined_model = os.path.join(self.path,self.combined_model_name)

if __name__ == "__main__":
    c = CommoditySettings("onion")