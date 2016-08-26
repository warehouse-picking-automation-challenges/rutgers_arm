import re
from array import *
from math import *
import random
import math
from subprocess import call

# call(["roslaunch", "build_graphs.launch"])

ITERATIONS = "2"
FIX_POSES = "rss2_poses"
TEST_NAME = "_toDelete"
NUM_POSES = 0
NUM_EXTRA_POSES = "0"
ENVIRONMENT = "table"
ADDITIONAL_OBJECTS = ""
IGNORE_LIST = ""
filename = "buildGraphs.launch"
ON_TABLE = "3.4775"
RETRACT_DIST = "0.04"
DIFFERENT_GRASPS = "1"
OBJECT_ORIENTATION = "[0,0,0,1]"
MIN_POSES = "0.95 , -0.1"
MAX_POSES = "1.13 , 0.6"
MIN_EXTRA_POSES = "0.8 , -0.08 , 3.27"
MAX_EXTRA_POSES = "1.15 , 0.58 , 3.4"


# ACP SETUP
# ON_TABLE = "3.2675"
# OBJECT_ORIENTATION = "[0,0,0,1]"
# MIN_POSES = "0.95 , -0.1"
# MAX_POSES = "1.13 , 0.6"
# MIN_EXTRA_POSES = "0.7 , -0.16 , 3.39"
# MAX_EXTRA_POSES = "0.85 , 0.64 , 3.9"

# min: [-0.455, 0.95]
# max: [0.455, 1.17]

# min: [-0.455, 0.85, 3.5]
# max: [0.455, 1.17, 3.6]

# object = "  <rosparam command=\"load\" ns=\"planning/world_model/simulator/subsystems/object#ID#\" file=\"$(find rearrangement_manipulation)/input/simulation/plants/cups.yaml\"/>\n  <rosparam ns=\"planning/world_model/simulator/subsystems/object#ID#\">\n    type: rigid_body_3d\n  </rosparam>\n"
object = "  <rosparam command=\"load\" ns=\"planning/world_model/simulator/subsystems/object#ID#\" file=\"$(find rearrangement_manipulation)/input/simulation/plants/cups.yaml\"/>\n"
pair = "          -\n            body_one: simulator/object#ID1#/body\n            body_two: simulator/object#ID2#/body\n"


NUM_FIX_POSES = 0 #Has to be the number of the fixed poses from the file. If the
#file is in the correct folder will update this number automatically 
fposes = open("planning/extra_poses/"+FIX_POSES+".yaml", "r")
for line in fposes:
    NUM_FIX_POSES = NUM_FIX_POSES + 1

NUM_FIX_POSES = NUM_FIX_POSES/2

fin = open("build_graphs_template.txt", "r")
fout = open(filename,"w")

IGNORE_LIST = "      ignore_list:\n        type: vector_collision_list\n        collision_list:\n"

for i in range(NUM_FIX_POSES+NUM_POSES):
    for j in range(i, NUM_FIX_POSES+NUM_POSES):
        if i != j:
            obj = pair 
            obj=re.sub('#ID1#', str(i), obj)
            obj=re.sub('#ID2#', str(j), obj)
            IGNORE_LIST = IGNORE_LIST + obj

for i in range(NUM_FIX_POSES+NUM_POSES):
    obj = object
    obj=re.sub('#ID#', str(i), obj)
    ADDITIONAL_OBJECTS = ADDITIONAL_OBJECTS + obj

for line in fin:
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
    line=re.sub('#ENVIRONMENT#', ENVIRONMENT, line)
    line=re.sub('#ADDITIONAL_OBJECTS#', ADDITIONAL_OBJECTS, line)
    line=re.sub('#IGNORE_LIST#', IGNORE_LIST, line)

    fout.write(line)
fout.close()
fin.close()
    
call(["roslaunch", "buildGraphs.launch"])