import re
from array import *
from math import *
import random
import math

SIM_NODE = "simulation"
PLAN_NODE = "planning"

test = "iros_table"
N = 1

REALISTIC_SCENARIO = "true"
FOR_TESTING = "true"
SIMULATE = FOR_TESTING
VISUALIZE = FOR_TESTING
PESISTENT = FOR_TESTING
GATHER_STATISTICS = "false"
ALGORITHM =       [ "NaiveAlg_JointBias",  "BatchAlg_JointBias",  "BatchAlg_AnyBias", "IncrementalAlg_ShortestSelection_AnyBias", "IncrementalAlg_LongestSelection_AnyBias", "IncrementalAlg_RandomSelection_AnyBias"]
SCHEDULING_ALGORITHM = [ "naive: true", "batch: true", "batch: true", "incremental: true", "incremental: true", "incremental: true" ]

SCHEDULING_ALGS = [ "RANDOM", "RANDOM", "RANDOM", "INCREMENTAL", "INCREMENTAL", "INCREMENTAL" ]
SELECTION_ALGS =  [ "NONE",   "NONE",   "NONE",   "SHORTEST",    "LONGEST",     "RANDOM" ]
BIAS_ALGS =       [ "JOINT",  "JOINT",  "ANY",    "ANY",         "ANY",         "ANY"]

NODE_FILENAMES =  [ "NAIVE_SHELF.txt",  "RANDOM_SHELF.txt",  "RANDOM_SHELF.txt", "INCREMENTAL_SHELF.txt", "INCREMENTAL_SHELF.txt", "RANDOM_SHELF.txt"]
K = [6,12, 18]

# THE FOLLOWING VARIABLES WILL AUTOMATICALLY BE FILLED

SIMULATION = "" #will have the text for the simulation if is active
VISUALIZATION = "" #will have the text for the visualization if is active
PLANNING = ""
MAPPING_LIST = ""        
OBJECT = ""
SAFE_STATE = ""
EXPERIMENT = ""
NUM_LEFT_PLANS = ""
NUM_RIGHT_PLANS = ""
LEFT_PLANS_PATH = ""
RIGHT_PLANS_PATH = ""
LEFT_ARM_TASKS = ""
RIGHT_ARM_TASKS = ""
NUM_OBJECTS = ""
PLANS_DIRECTORY = ""
CONSTRAINTS_DIRECTORY = ""

right_arm_color = "red"
left_arm_color = "blue"

BAXTER_TRANSLATION = ""
BAXTER_ROTATION = "" 
BAXTER = ""  

if test == "iros_shelf":
    BAXTER_TRANSLATION = "[0,0.255,0.785]"
    BAXTER_ROTATION = "[1.0,0,0,0,1.0,0,0,0,1]" 
    BAXTER = "_shifted"  
    EXPERIMENT = "IROS_SHELF"
    ENVIRONMENT = "iros_shelf"    
    LEFT_SAFE_STATE = "[1.5681561256801836989893673,-0.6933267886173464233934283,-0.8849957765343199334040492,1.9000335039215263677192524,-2.2107340224187499444497007,-0.8368545446250841290947164,0.9573491235931878007292539,0.0000000000000000000000000]"
    RIGHT_SAFE_STATE = "[-1.6812774859126198290226739,-0.5125644883535177553923745,1.0773961507954412653020881,1.8738453488311539452126908,1.9879612960600967497271085,-0.9987250827246928475133814,-0.8436528393489071131483570,0.0000000000000000000000000]"
    LEFT_ARM_DROPOFF = "[0.00000000,0.80000000,0.80000000,0.70710678,0.00000000,-0.70710678,0.00000000]"
    RIGHT_ARM_DROPOFF = "[0.00000000,-0.80000000,0.80000000,0.70710678,0.00000000,-0.70710678,0.00000000]"
    NUM_LEFT_PLANS = "22"
    NUM_RIGHT_PLANS = "22"
    PLANS_DIRECTORY = "/prx_packages/coordination_manipulation/input/experiments/iros_shelf/plans/"
    CONSTRAINTS_DIRECTORY = "/prx_packages/coordination_manipulation/input/experiments/iros_shelf/coordination_constraints/"

    left_poses_filename = "poses/all_poses_iros_shelf_left.txt"
    right_poses_filename = "poses/all_poses_iros_shelf_right.txt"
    common_poses_filename = "poses/all_poses_iros_shelf_common.txt"
    
    clear_dist = 0.04

    RAD = "0.02"  
    HEIGHT = "0.14"

    fobject = open("templates/cup.txt","r")
    for line in fobject:
        line=re.sub('#RAD#', RAD, line)
        line=re.sub('#HEIGHT#', HEIGHT, line)
        line=re.sub('#COLOR#', "blue", line)
        line=re.sub('#INITIAL_STATE#', "", line)
        OBJECT = OBJECT + line   

if SIMULATE == "true":
    fsim = open("templates/coordination_simulation.txt","r")
    for line in fsim.readlines():
        line=re.sub('#NUM_OBJECTS#', NUM_OBJECTS, line)
        line=re.sub('#LEFT_SAFE_STATE#',LEFT_SAFE_STATE, line)
        line=re.sub('#RIGHT_SAFE_STATE#',RIGHT_SAFE_STATE, line)
        line=re.sub('#LEFT_ARM_DROPOFF#',LEFT_SAFE_STATE, line)
        line=re.sub('#RIGHT_ARM_DROPOFF#',RIGHT_SAFE_STATE, line)
        line=re.sub('#ENVIRONMENT#', ENVIRONMENT, line)
        line=re.sub('#PLAN_NODE#', PLAN_NODE, line)
        line=re.sub('#BAXTER#', BAXTER, line)
        line=re.sub('#BAXTER_ROTATION#', BAXTER_ROTATION, line)
        line=re.sub('#BAXTER_TRANSLATION#', BAXTER_TRANSLATION, line)
        SIMULATION = SIMULATION + line

if VISUALIZE == "true":    
    fvis = open("templates/visualization.txt","r")

    for line in fvis.readlines():
        VISUALIZATION = VISUALIZATION + line

if FOR_TESTING == "true":
    PLANNING = "<node name=\"planning\" pkg=\"prx_planning\" type=\"prx_planning\" required=\"true\" launch-prefix=\"gdb --args\" output=\"screen\" args=\"" + PLAN_NODE + "\"/>"


left_final_poses = open(left_poses_filename,"r")
right_final_poses = open (right_poses_filename,"r")
all_left_poses = []
all_right_poses = []
final_poses = []
poses = []
used_poses = []
selected_left_poses = []
selected_right_poses = []
objects = ""
plan_objects = ""
object_pairs = ""

for line in left_final_poses.readlines():
    line = line.rstrip('\n')
    all_left_poses.append(line)

for line in right_final_poses.readlines():
    line = line.rstrip('\n')
    all_right_poses.append(line)

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


def build_object(i, p, color_to_use):
    global objects
    global plan_objects
    global MAPPING_LIST
    
    if(SIMULATE == "true"):
        obj = OBJECT
        obj=re.sub('#NODE#', SIM_NODE, obj)
        obj=re.sub('#NAME#', "cup", obj)
        obj=re.sub('#ID#', str(i+1), obj)
        obj=re.sub('#INITIAL_STATE#', "initial_state: [" + p + "]", obj)
        obj=re.sub('#COLOR#', color_to_use, obj)
        objects = objects + obj + "\n\n"

    objP = OBJECT
    objP=re.sub('#NAME#', "cup", objP)
    objP=re.sub('#ID#', str(i+1), objP)
    objP=re.sub('#INITIAL_STATE#', "", objP)
    objP=re.sub('#COLOR#', color_to_use, objP)
    
    plan_objects = plan_objects + objP + "\n\n"

def build_problem(k):
    global test
    global random_left_poses
    global random_right_poses
    global selected_right_poses
    global selected_left_poses
    global s_poses
    global f_poses
    global objects
    global plan_objects
    global used_poses
    global MAPPING_LIST

    random_left_poses = [] 
    random_right_poses = []  
    selected_left_poses = []
    selected_right_poses = []
    for index, item in enumerate(all_left_poses):
        print "Read in LEFT pose:", index, item, "\n"
        random_left_poses.append((index,all_left_poses[index])) 
    for index, item in enumerate(all_right_poses):
        print "Read in RIGHT pose:", index, item, "\n"
        random_right_poses.append((index,all_right_poses[index])) 
    #poses.extend(all_poses)
    used_poses = []
    objects = ""
    plan_objects = ""
    MAPPING_LIST = ""

    for i in range(k):
        print(i)
        
        
        if i < (k/2):
            p = random.choice(random_right_poses)
            if (REALISTIC_SCENARIO == "true"):
                while(collision_free_pose(p[1]) == False):
                    random_right_poses.remove(p)
                    if len(random_right_poses) == 0:
                        return False
                    p = random.choice(random_right_poses)
            build_object(i,p[1],right_arm_color)
            selected_right_poses.append(p[0])
            random_right_poses.remove(p)
        else:
            p = random.choice(random_left_poses)
            if (REALISTIC_SCENARIO == "true"):
                while(collision_free_pose(p[1]) == False):
                    random_left_poses.remove(p)
                    if len(random_left_poses) == 0:
                        return False
                    p = random.choice(random_left_poses)
            build_object(i,p[1], left_arm_color)
            selected_left_poses.append(p[0])
            random_left_poses.remove(p)

    print(str(i) +  "  TRUE  for k : " + str(k))
    return True

for k in K:
    for n in range(N):                
        while build_problem(k) == False:
            print("Could not find problem for : _" + test + "_" + str(k))
        n_str = str(n);
        k_str = str(k)
        if n < 10:
            n_str = "0"+n_str;
        if k < 10:
            k_str = "0"+k_str;
        for sched_alg in enumerate(ALGORITHM):
            alg_selection = sched_alg[0]
            SCHEDULING_TYPE = SCHEDULING_ALGS[alg_selection]
            SELECTION_TYPE = SELECTION_ALGS[alg_selection]
            BIAS_TYPE = BIAS_ALGS[alg_selection]
            ALG_TYPE = SCHEDULING_ALGORITHM[alg_selection]
            NODE_FILE = NODE_FILENAMES[alg_selection]
            fin = open("templates/solve_constraints_template.txt", "r")
            filename = "launches/"+test+"_"+sched_alg[1]+"_"+k_str+"k_"+n_str+".launch"
            RESULTS_FILE = test+"_"+sched_alg[1]+"_"+k_str+"k_results.txt"
            if FOR_TESTING != "true":
                PLAN_NODE = test+"_"+sched_alg[1]+"_"+str(k)+"k_"+n_str+"planning"
            fout2 = open(NODE_FILE, 'a')
            fout2.write(PLAN_NODE + "\n")
            fout2.close()
            fout = open(filename, "w");
            for line in fin.readlines():
                NUM_OBJECTS = str(k/2);
                LEFT_ARM_TASKS = "["
                for i in range(k/2):
                    LEFT_ARM_TASKS += (str(selected_left_poses[i]) + ",");
                LEFT_ARM_TASKS = LEFT_ARM_TASKS[:-1] + "]"
                RIGHT_ARM_TASKS = "["
                for i in range(k/2):
                    RIGHT_ARM_TASKS += (str(selected_right_poses[i]) + ",");
                RIGHT_ARM_TASKS = RIGHT_ARM_TASKS[:-1] + "]"
                line=re.sub('#PLAN_NODE#', PLAN_NODE, line)
                line=re.sub('#RESULTS_FILE#', RESULTS_FILE, line)
                line=re.sub('#SCHEDULE_ALGORITHM#', ALG_TYPE, line)
                line=re.sub('#EXPERIMENT#', EXPERIMENT, line)
                line=re.sub('#NUM_LEFT_PLANS#', NUM_LEFT_PLANS, line)
                line=re.sub('#NUM_RIGHT_PLANS#', NUM_RIGHT_PLANS, line)
                line=re.sub('#PLANS_DIRECTORY#', PLANS_DIRECTORY, line)
                line=re.sub('#CONSTRAINTS_DIRECTORY#', CONSTRAINTS_DIRECTORY, line)
                line=re.sub('#LEFT_ARM_TASKS#', LEFT_ARM_TASKS, line)
                line=re.sub('#RIGHT_ARM_TASKS#', RIGHT_ARM_TASKS, line)

                line=re.sub('#BAXTER#', BAXTER, line)
                line=re.sub('#BAXTER_ROTATION#', BAXTER_ROTATION, line)
                line=re.sub('#BAXTER_TRANSLATION#', BAXTER_TRANSLATION, line)

                line=re.sub('#SCHEDULING_TYPE#', SCHEDULING_TYPE, line)
                line=re.sub('#SELECTION_TYPE#', SELECTION_TYPE , line)
                line=re.sub('#BIAS_TYPE#', BIAS_TYPE, line)

                line=re.sub('#LEFT_SAFE_STATE#',LEFT_SAFE_STATE, line)
                line=re.sub('#RIGHT_SAFE_STATE#',RIGHT_SAFE_STATE, line)

                line=re.sub('#ENVIRONMENT#', ENVIRONMENT, line)
                line=re.sub('#SIMULATE#', SIMULATE, line)
                line=re.sub('#VISUALIZE#', VISUALIZE, line)
                line=re.sub('#PERSISTENT#', PESISTENT, line)

                line=re.sub('#OBJECTS#', objects, line)
                line=re.sub('#SIMULATION#', SIMULATION, line)
                line=re.sub('#VISUALIZATION#', VISUALIZATION, line)        
                line=re.sub('#PLANNING#', PLANNING, line)                            
                fout.write(line)
        fin.close()

print("DONE")
