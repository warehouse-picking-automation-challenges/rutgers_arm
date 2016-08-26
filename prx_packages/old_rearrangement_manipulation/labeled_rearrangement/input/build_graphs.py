import re
from array import *
from math import *
import random
import math
from subprocess import call

# call(["roslaunch", "build_graphs.launch"])
PACKAGE = "labeled_rearrangement"
PLANNING_NODE = "graph_builder_planning"
test = "motoman"
ROBOT = "motoman"

ITERATIONS = "1500"
NUM_EXTRA_POSES = "0"
NUM_POSES = 0
OBJECT = ""

GRASP_Z = "0"
GRASP_X = "0"
MOTOMAN_SAFE_STATE = "[0 1.57 0 0 -1.70 0 0 0 1.57 0 0 -1.70 0 0 0 0 0]"

if test == "motoman":
    TEST_NAME = "rss_" 
    BAXTER = "_side"
    FIX_POSES = "motoman_rss_poses"       
    ENVIRONMENT = "rss_table_setup"
    ON_TABLE = "0.575"
    OBJECT_ORIENTATION = "[0, 0, 0, 1]"
    MIN_POSES = "0.7, -0.60"
    MAX_POSES = "1.10, 0.60 "
    MIN_EXTRA_POSES = "0.7, -0.60, 0.95"
    MAX_EXTRA_POSES = "1.10, 0.6, 1.2"    
    MIN_SAMPLE_POSES = "-0.5, 0.6, 0.60, 0, 0, 0, 0"
    MAX_SAMPLE_POSES = "0.5, 1.02, 1, 1, 1, 1, 1"
    MIN_GRASP_THETA = "0"
    MAX_GRASP_THETA = "0"
    MAX_GRASP_TRIES = "50"
    RELATIVE_CONFIGURATION = "[-0.0268,0,0,0.5,0.5,0.5,0.5]"
    GRASP_X = "-0.027"
    RETRACT_DIST = "0.1"
    DIFFERENT_GRASPS = "1"
    BOXES_ENV = "boxes_env: true"
    DIM = "0.055 , 0.03 , 0.12"  

    fobject = open("templates/box.txt","r")
    for line in fobject:    
        line=re.sub('#DIM#', DIM, line)
        line=re.sub('#COLOR#', "blue", line)
        line=re.sub('#INITIAL_STATE#', "", line)
        OBJECT = OBJECT + line
elif test == "rss":
    TEST_NAME = "table_" 
    BAXTER = "_side"
    FIX_POSES = "table_poses"       
    ENVIRONMENT = "rss_table"
    ON_TABLE = "0.86"
    if ROBOT == "motoman":
        OBJECT_ORIENTATION = "[-0.5,0.5,-0.5,0.5]"
    else:        
        OBJECT_ORIENTATION = "[0.7071067811865476,0,-0.7071067811865475,0]"
    MIN_POSES = "0.7, -0.60"
    MAX_POSES = "1.10, 0.60 "
    MIN_EXTRA_POSES = "0.7, -0.60, 0.95"
    MAX_EXTRA_POSES = "1.10, 0.6, 1.2"
    MIN_GRASP_THETA = "0"
    MAX_GRASP_THETA = "3.14159"
    MAX_GRASP_TRIES = "2"
    RETRACT_DIST = "0.05"
    DIFFERENT_GRASPS = "10"
    BOXES_ENV = "boxes_env: true"
    DIM = "0.055 , 0.03 , 0.12"  

    fobject = open("templates/box.txt","r")
    for line in fobject:    
        line=re.sub('#DIM#', DIM, line)
        line=re.sub('#COLOR#', "blue", line)
        line=re.sub('#INITIAL_STATE#', "", line)
        OBJECT = OBJECT + line
elif test == "beer":
    NUM_POSES = 0
    BAXTER = ""  
    TEST_NAME = "beer_"
    FIX_POSES = "beer_poses"       
    ENVIRONMENT = "beer_setup"
    ON_TABLE = "1.06"
    OBJECT_ORIENTATION = "[0.00,0.00,0.00,1]"
    MIN_POSES = "0.7, -0.60"
    MAX_POSES = "1.10, 0.60 "
    MIN_EXTRA_POSES = "0.75, -0.1, 1.07"
    MAX_EXTRA_POSES = "1.13, 0.6, 1.12"
    MIN_GRASP_THETA = "0"
    MAX_GRASP_THETA = "0"
    MAX_GRASP_TRIES = "2"
    RETRACT_DIST = "0.07"
    DIFFERENT_GRASPS = "1"
    BOXES_ENV = "boxes_env: false"
    RAD = "0.02"  
    HEIGHT = "0.14"

    fobject = open("templates/cup.txt","r")
    for line in fobject:
        line=re.sub('#RAD#', RAD, line)
        line=re.sub('#HEIGHT#', HEIGHT, line)
        line=re.sub('#COLOR#', "blue", line)
        line=re.sub('#INITIAL_STATE#', "", line)
        OBJECT = OBJECT + line
elif test == "hanoi":
    NUM_POSES = 0
    BAXTER = ""  
    TEST_NAME = "hanoi_"
    FIX_POSES = "hanoi_poses"       
    ENVIRONMENT = "hanoi_kiva_pod"
    ON_TABLE = "1.06"
    OBJECT_ORIENTATION = "[0.00,0.00,0.00,1]"
    MIN_POSES = "0.70, 0.0"
    MAX_POSES = "1.10, 0.60 "
    MIN_EXTRA_POSES = "0.80, 0.2, 1.08"
    MAX_EXTRA_POSES = "1.10, 0.6, 1.35"
    MIN_GRASP_THETA = "3.14159"
    MAX_GRASP_THETA = "3.14159"
    MAX_GRASP_TRIES = "1"
    RETRACT_DIST = "0.35"
    DIFFERENT_GRASPS = "1"
    BOXES_ENV = "boxes_env: false"
    RAD = "0.02"  
    HEIGHT = "0.14"

    fobject = open("templates/cup.txt","r")
    for line in fobject:
        line=re.sub('#RAD#', RAD, line)
        line=re.sub('#HEIGHT#', HEIGHT, line)
        line=re.sub('#COLOR#', "blue", line)
        line=re.sub('#INITIAL_STATE#', "", line)
        OBJECT = OBJECT + line
else:    
    BAXTER = ""    
    FIX_POSES = "shelf_poses"
    TEST_NAME = "shelf_"
    ENVIRONMENT = "shelf"
    ON_TABLE = "1.4775"
    #OBJECT_ORIENTATION = "[0.00,0.00,0.7071067811800000324495841,0.7071067811800000324495841]"
    OBJECT_ORIENTATION = "[0.00,0.00,0.00,1.00]"
    MIN_POSES = "-0.455, 0.95"
    MAX_POSES = "0.455, 1.17 "
    MIN_EXTRA_POSES = "-0.4, 0.75 , 1.5"
    MAX_EXTRA_POSES = "0.4, 1.17, 1.6"
    MIN_GRASP_THETA = "3.64"
    MAX_GRASP_THETA = "5.78"
    MAX_GRASP_TRIES = "20"
    RETRACT_DIST = "0.05"
    DIFFERENT_GRASPS = "5"
    BOXES_ENV = "boxes_env: false"
    RAD = "0.02"  
    HEIGHT = "0.14"

    fobject = open("templates/cup.txt","r")
    for line in fobject:
        line=re.sub('#RAD#', RAD, line)
        line=re.sub('#HEIGHT#', HEIGHT, line)
        line=re.sub('#COLOR#', "blue", line)
        line=re.sub('#INITIAL_STATE#', "", line)
        OBJECT = OBJECT + line

ADDITIONAL_OBJECTS = ""
IGNORE_LIST = ""
filename = "new_graphs.launch"

BAXTER_ARM = "left"
if BAXTER == "_side" :
    BAXTER_ROTATION = "[0,1.0,0,-1.0,0,0,0,0,1]"
    BAXTER_TRASLATION = "[0,0,0.785]"
else:
    BAXTER_ROTATION = "[1.0,0,0,0,1.0,0,0,0,1]"
    BAXTER_TRASLATION = "[0,0,0.785]"

pair = "          -\n            body_one: simulator/object#ID1#/body\n            body_two: simulator/object#ID2#/body\n"

NUM_FIX_POSES = 0 #Has to be the number of the fixed poses from the file. If the
#file is in the correct folder will update this number automatically 
fposes = open("poses/"+FIX_POSES+".yaml", "r")
for line in fposes:
    NUM_FIX_POSES = NUM_FIX_POSES + 1
NUM_FIX_POSES = NUM_FIX_POSES/2

if ROBOT == "baxter":
    fin = open("templates/build_graphs_template.txt", "r")
else:
    fin = open("templates/build_graphs_motoman_template.txt", "r")
fout = open(filename,"w")

IGNORE_LIST = "      ignore_list:\n        type: vector_collision_list\n        collision_list:\n"

for i in range(NUM_FIX_POSES+NUM_POSES):
    for j in range(i, NUM_FIX_POSES+NUM_POSES):
        if i != j:
            obj = pair 
            obj=re.sub('#ID1#', str(i), obj)
            obj=re.sub('#ID2#', str(j), obj)
            IGNORE_LIST = IGNORE_LIST + obj

CUP = OBJECT
CUP = re.sub('#NODE#', PLANNING_NODE + "/world_model", CUP)
CUP = re.sub('#NAME#', "cup", CUP)
CUP = re.sub('#ID#', "", CUP)

for i in range(NUM_FIX_POSES+NUM_POSES):
    obj = OBJECT
    obj = re.sub('#NODE#', PLANNING_NODE + "/world_model", obj)
    obj = re.sub('#NAME#', "object", obj)
    obj = re.sub('#ID#', str(i), obj)
    ADDITIONAL_OBJECTS = ADDITIONAL_OBJECTS + "\n\n" + obj 

for line in fin:
    line=re.sub('#PACKAGE#', PACKAGE, line)
    line=re.sub('#PLANNING_NODE#', PLANNING_NODE, line)
    line=re.sub('#SAFE_STATE#', MOTOMAN_SAFE_STATE, line)
    line=re.sub('#BAXTER#', BAXTER, line)
    line=re.sub('#BAXTER_ARM#', BAXTER_ARM, line)
    line=re.sub('#BAXTER_ROTATION#', BAXTER_ROTATION, line)
    line=re.sub('#BAXTER_TRASLATION#', BAXTER_TRASLATION, line)
    line=re.sub('#ITERATIONS#', ITERATIONS, line)
    line=re.sub('#FIX_POSES#', FIX_POSES, line)
    line=re.sub('#TEST_NAME#', TEST_NAME, line)
    line=re.sub('#NUM_POSES#', str(NUM_POSES), line)
    line=re.sub('#NUM_EXTRA_POSES#', NUM_EXTRA_POSES, line)
    line=re.sub('#ON_TABLE#', ON_TABLE, line)
    line=re.sub('#RETRACT_DIST#', RETRACT_DIST, line)
    line=re.sub('#DIFFERENT_GRASPS#', DIFFERENT_GRASPS, line)
    line=re.sub('#OBJECT_ORIENTATION#', OBJECT_ORIENTATION, line)
    line=re.sub('#MIN_POSES#', MIN_POSES, line)
    line=re.sub('#MAX_POSES#', MAX_POSES, line)
    line=re.sub('#MIN_EXTRA_POSES#', MIN_EXTRA_POSES, line)
    line=re.sub('#MAX_EXTRA_POSES#', MAX_EXTRA_POSES, line)
    line=re.sub('#MIN_SAMPLE_POSES#', MIN_SAMPLE_POSES, line)
    line=re.sub('#MAX_SAMPLE_POSES#', MAX_SAMPLE_POSES, line)    
    line=re.sub('#MIN_GRASP_THETA#', MIN_GRASP_THETA, line)
    line=re.sub('#MAX_GRASP_THETA#', MAX_GRASP_THETA, line)
    line=re.sub('#MAX_GRASP_TRIES#', MAX_GRASP_TRIES, line)
    line=re.sub('#RELATIVE_CONFIGURATION#', RELATIVE_CONFIGURATION, line)
    line=re.sub('#GRASP_X#', GRASP_X, line)
    line=re.sub('#GRASP_Z#', GRASP_Z, line)
    line=re.sub('#ENVIRONMENT#', ENVIRONMENT, line)
    line=re.sub('#CUP#', CUP, line)    
    line=re.sub('#ADDITIONAL_OBJECTS#', ADDITIONAL_OBJECTS, line)
    line=re.sub('#IGNORE_LIST#', IGNORE_LIST, line)
    line=re.sub('#BOXES_ENV#', BOXES_ENV, line)        
    fout.write(line)
fout.close()
fin.close()
    
call(["roslaunch", "new_graphs.launch"])
