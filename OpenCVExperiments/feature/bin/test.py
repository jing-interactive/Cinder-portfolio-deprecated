import os
import sys
import glob
import shutil

ukbenck_part_1 = 'D:\\snda_projects\\ImageRetrieval\\data\\ukbenck\\1\\'
ukbenck_part_2 = 'D:\\snda_projects\\ImageRetrieval\\data\\ukbenck\\2\\'
ukbenck_part_3 = 'D:\\snda_projects\\ImageRetrieval\\data\\ukbenck\\3\\'
ukbenck_part_4 = 'D:\\snda_projects\\ImageRetrieval\\data\\ukbenck\\4\\'

CMD_BASE = '..\\bin\\Release\\ExtractSiftFromSingleImage.exe '

def extract_dir(img_dir):
    for file in glob.glob(img_dir + '\\*.jpg'):
        print file
        key_file_name = file + '.sift.key'
        desc_file_name = file + '.sift.desc'
        #print CMD_BASE + file + ' ' + key_file_name + ' ' + desc_file_name
        os.system(CMD_BASE + file + ' ' + key_file_name + ' ' + desc_file_name)
        
        
extract_dir(ukbenck_part_1) 