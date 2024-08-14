# creates a csv file for a mock run
import os,sys,csv


class LogParser(object):
    def __init__(self):
        self.path = "/home/occipital/.ros/log"
        self.base_path = "./metadata"
        self.files = []
        self.printList = []
        self.csv_array = []
        self.headers = ["timestamp","attribute",'camera sequence','camera frame id','camera stamp','process sequence','process frame id','process stamp','X','Y','prior_X','prior_Y','defect','color','decision sequence','decision frame id','decision stamp','decision A','decision B','decision C','signal sequence','signal frame id','signal stamp','signal A','signal B','signal C']
        self.data = {
            "cam_log" : [],
            "proc_log": [],
            "gen_sig_log": [],
            "out_sig_log": []
                }

        latest_path = self.get_latest_log_file(self.path) # got path for latest generated path
        print("Opened File {}".format(latest_path))
        self.read_lines_from_file(latest_path) #read lines from the file
        self.loop_over_lines() # loop over to extract each line for preprocessing
        self.preprocess_lines() # literally segregating each variable
        self.write_data_toCSV() # writing in csv

    def get_latest_log_file(self,path):
        for i in os.listdir(path):
            if i.startswith("data_recording_node"):
                self.files.append(os.path.join(path,i))
        return max(self.files,key = os.path.getctime)

    def read_lines_from_file(self,latest_path):
        self.filename_of_log = os.path.basename(latest_path).split('.')[0]
        theFile = open(latest_path,'r')
        self.FILE = theFile.readlines()
        theFile.close()

    def loop_over_lines(self):
        for line in self.FILE:
            for i in (self.data.keys()):
                if (i in line) or ('Totals' in line):
                    self.printList.append(line.strip("[rosout][INFO] \n :").split(',',11))
                    
    
    def preprocess_lines(self):
        
        for line in self.printList:
            self.arrays = ["Nan"]*26
            self.arrays[0] = line[0] +":"+ line[1].split(':')[0]
            self.arrays[1] = line[1].split(':')[1] 
            if line[1].endswith('cam_log') :
                self.arrays[2:4] = line[2:]
            elif line[1].endswith('proc_log') :
                self.arrays[5:13] = line[2:]
            elif line[1].endswith('gen_sig_log'):
                self.arrays[14:19] = line[2:]
            elif line[1].endswith('out_sig_log'):
                self.arrays[20:] = line[2:]

            self.csv_array.append(self.arrays)

    def write_data_toCSV(self):
        with open(self.base_path +"/"+'output_'+ self.filename_of_log +'.csv','w') as f:
            writer = csv.writer(f)
            
            writer.writerow(self.headers)
            writer.writerows(self.csv_array)




if __name__ == '__main__':
    LogParser()