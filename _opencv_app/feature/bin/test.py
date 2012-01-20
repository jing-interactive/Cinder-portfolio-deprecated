import os
import sys
import glob
import shutil, subprocess
import random
import config

EXE = './Feature.exe'

class Folder:
    def __init__(self, folder_name, filter):
        self.name = folder_name
        self.files = []
        for file in glob.glob(folder_name+filter):
            self.files.append(file)
            
    def printf(self):
        print self.files 
        
    def size(self):
        return len(self.files)
        
    def get_random_file(self):
        return self.files[random.randint(0, self.size()-1)]

def compare(left_file, right_file, params, should_equal):
    '''compare two images to decide see if they should_be_equal'''
    p = subprocess.Popen([EXE, left_file, right_file, params], stdout=subprocess.PIPE)
    p.wait()
    if p.returncode < 0:
        print 'error occurs', EXE,left_file, right_file, params
        return
    #print should_be_equal, p.returncode
    if should_equal:
        config.true_total = config.true_total+1
        if p.returncode == 0:
            print should_equal, p.returncode, EXE,left_file, right_file, params
            config.true_negative = config.true_negative+1
    elif not should_equal:
        config.false_total = config.false_total+1
        if p.returncode > 0:
            print should_equal, p.returncode, EXE,left_file, right_file, params
            config.false_positive = config.false_positive+1

def run_test(testset_name, params):
    '''run a full test on testset_name'''
    config.false_total = 0
    config.true_total = 0
    config.false_positive = 0
    config.true_negative = 0 
    
    folder_list = [] 
    list = os.listdir(testset_name)
    for l in list:
        entry = Folder(testset_name+'/'+l, '/*.*')
        #entry.printf()
        folder_list.append(entry)
        
    n_folders = len(folder_list)

    for i in range(config.n_positve_tests):#positive tests
        folder_a = random.randint(0, n_folders-1) 
        file_a = folder_list[folder_a].get_random_file()
        file_b = folder_list[folder_a].get_random_file()
        compare(file_a, file_b, params, True)
        
    for i in range(config.n_negative_tests):#negative tests
        folder_a = random.randint(0, n_folders/2)
        folder_b = random.randint(n_folders/2+1, n_folders-1)
        file_a = folder_list[folder_a].get_random_file()
        file_b = folder_list[folder_b].get_random_file()
        compare(file_a, file_b, params, False)

    print "true_positive rate = ",(config.true_total-config.true_negative*1.0)/config.true_total
    print "false_negative rate = ",(config.false_total-config.false_positive*1.0)/config.false_total

if __name__ == '__main__':
    run_test('Cambridge','')
    run_test('FERET','')