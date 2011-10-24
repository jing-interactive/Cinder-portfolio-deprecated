import os
import sys
import glob
import shutil

CMD_BASE = 'Feature.exe'

class Folder:
    def __init__(self, folder_name, filter):
        self.name = folder_name
        self.files = []
        for file in glob.glob(folder_name+filter):
            self.files.append(file)
            
    def printf(self):
        print self.files
        
folder_list = []
for i in range(1,40): 
    entry = Folder('Cambridge_FaceDB/s'+str(i), '/*.*')
    #entry.printf()
    folder_list.append(entry)

for entry in folder_list:
    os.system(CMD_BASE + ' '+entry.files[0] + ' ' + entry.files[2])
    os.system(CMD_BASE + ' '+entry.files[2] + ' ' + entry.files[4])
    os.system(CMD_BASE + ' '+entry.files[1] + ' ' + entry.files[3])
    os.system(CMD_BASE + ' '+entry.files[0] + ' ' + entry.files[3])