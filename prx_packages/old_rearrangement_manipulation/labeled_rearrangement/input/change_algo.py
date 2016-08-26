import re
from array import *
from math import *
import random
import math
import os, os.path
import shutil

DIRS = ["launchesShelf"]
algo = "type: mrs"
replace_with = "type: fmrs"

for dir_path in DIRS:
    in_dir = dir_path
    out_dir = dir_path+"fmrs"
    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)
    else:
        shutil.rmtree(out_dir)
        os.makedirs(out_dir)
    files_in_dir = os.listdir(in_dir)
    for file_in_dir in files_in_dir:
        filename = in_dir + "/" + file_in_dir
        if(os.path.isfile(filename)):
            filename_out = out_dir+"/"+file_in_dir
            fin = open(filename, "r")
            fout = open(filename_out,"w")
            print(filename)
            for line in fin.readlines():
                line=re.sub(algo, replace_with, line)
                fout.write(line)
            fin.close()
            fout.close()
