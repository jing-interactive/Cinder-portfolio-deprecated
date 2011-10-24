import os
import sys
import glob
import shutil

CMD_BASE = 'Feature.exe'

folder_list = []
for i in range(1,40):
    folder_list.append({})
    folder_list[i].folder = 's'+str(i);
    folder_list[i].files = []  
    for file in glob.glob('s'+str(i)+'/*.*'):
        folder_list[i].files.append(file)
    print folder_list[i]

def extract_dir(img_dir):
    for file in glob.glob(img_dir + '\\*.jpg'):
        print file
        key_file_name = file + '.sift.key'
        desc_file_name = file + '.sift.desc'
        #print CMD_BASE + file + ' ' + key_file_name + ' ' + desc_file_name
        os.system(CMD_BASE + file + ' ' + key_file_name + ' ' + desc_file_name)
        

     

