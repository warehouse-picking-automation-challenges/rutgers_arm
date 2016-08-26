import re
from array import *
from math import *
import random
import math

Colors=["red", "green", "blue", "yellow", "pink", "grey", "black", "purple", "orange", "light_blue"]
RSSColors=["red1", "red2","red3","red4","red5","red6", "green1", "green2", "green3", "green4", "green5", "blue1", "blue2", "blue3", "blue4", "blue5"]
# K = [2,4,6,8,10]
K = [6,8]
B = [0]
N = 1
clear_dist = 0.1
object_filename = "cup.txt"
#object_filename = "box.txt"
fin_final_poses = open("grid_poses.txt","r")
fin_poses = open ("poses.txt","r")
add_final_poses_to_poses = False
all_poses = []
final_poses = []
poses = []
used_poses = []
s_poses = ""
f_poses = ""
objects = ""
FOLDER = "rearrangement_manipulation"
# FOLDER = "labeled_rearrangement"


def calc_dist(p1,p2):
    return math.sqrt(math.pow((p2[0] - p1[0]), 2) +
                     math.pow((p2[1] - p1[1]), 2) +
                     math.pow((p2[2] - p1[2]), 2))

def collision_free_pose(p):
    p_split = p.split(',') 
    p_point = (float(p_split[0]), float(p_split[1]), float(p_split[2])) 
    for pose in used_poses:
        if calc_dist(p_point,pose)  < clear_dist:
            return False

    used_poses.append(p_point)
    return True


def build_object(i, p):
    global object_filename
    global objects
    
    fin_object = open(object_filename,"r")
    for line in fin_object:
        line=re.sub('#ID#', str(i+1), line)
        line=re.sub('#POSE#', p, line)
        line=re.sub('#COLOR#', Colors[i], line)
        objects = objects + line
    objects = objects + "\n\n"     
    fin_object.close()   


def build_problem(k):
    global poses
    global s_poses
    global f_poses
    global objects
    global used_poses

    poses = []    
    poses.extend(all_poses)
    used_poses = []
    s_poses = ""
    f_poses = ""
    objects = ""

    for i in range(k):    
        p = random.choice(poses)
        while(collision_free_pose(p) == False):
            poses.remove(p)
            if len(poses) == 0:
                return False
            p = random.choice(poses)
            
         
        s_poses = s_poses + "      -\n" + "        pose: [" + p + "]\n" 
        f_poses = f_poses + "      -\n" + "        pose: [" + final_poses[i] + "]\n"
        build_object(i,p)
        poses.remove(p)

    return True

for line in fin_poses.readlines():
    line = line.rstrip('\n')
    all_poses.append(line)

for line in fin_final_poses.readlines():
    line = line.rstrip('\n')
    final_poses.append(line)

if(add_final_poses_to_poses):
    all_poses.extend(final_poses)


for k in K:
    for b in B:
        for n in range(N):            
            if build_problem(k) == True:
                fin = open("template.txt", "r")
                n_str = str(n);
                if n < 10:
                    n_str = "0"+n_str;
                filename = "launches/"+str(k)+"k_"+str(b)+"b_"+n_str+".launch"
                fout = open(filename, "w");
                for line in fin.readlines():
                    line=re.sub('#Kval#', str(k), line)
                    line=re.sub('#Bval#', str(b), line)
                    line=re.sub('#SPOSES#', s_poses, line)
                    line=re.sub('#FPOSES#', f_poses, line)
                    line=re.sub('#OBJECTS#', objects, line)
                    fout.write(line)
            fin.close()

