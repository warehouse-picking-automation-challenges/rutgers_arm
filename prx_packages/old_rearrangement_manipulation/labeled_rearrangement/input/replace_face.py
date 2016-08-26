import re
from array import *
from math import *
import random
import math
import os, os.path
import shutil

DIRS = ["launchesRSS", "launchesGrid", "launchesGrid2", "launchesShelf"]

for dir_path in DIRS:
    in_dir = dir_path+"old"
    if not os.path.isdir(dir_path):
        os.makedirs(dir_path)
    else:
        shutil.rmtree(dir_path)
        os.makedirs(dir_path)
    files_in_dir = os.listdir(in_dir)
    for file_in_dir in files_in_dir:
        filename = in_dir + "/" + file_in_dir
        if(os.path.isfile(filename)):
            filename_out = dir_path+"/"+file_in_dir
            fin = open(filename, "r")
            fout = open(filename_out,"w")
            print(filename)
            for line in fin.readlines():
                line=re.sub("face", "phase", line)
                line=re.sub("poses_shelf.txt", "shelf_poses.txt", line)
                line=re.sub("informed_transit_shelf.txt", "shelf_informed_transit.txt", line)
                line=re.sub("informed_transfer_shelf.txt", "shelf_informed_transfer.txt", line)
                line=re.sub("poses_rss.txt", "rss_poses.txt", line)
                line=re.sub("informed_transit_rss.txt", "rss_informed_transit.txt", line)
                line=re.sub("informed_transfer_rss.txt", "rss_informed_transfer.txt", line)
                line=re.sub("poses_grid.txt", "grid_poses.txt", line)
                line=re.sub("informed_transit_grid.txt", "grid_informed_transit.txt", line)
                line=re.sub("informed_transfer_grid.txt", "grid_informed_transfer.txt", line)
                fout.write(line)
            fin.close()
            fout.close()
