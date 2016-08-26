import re
from array import *
from math import *
import random
import math
import os, os.path
import shutil

PACKAGE = "labeled_rearrangement"
PLAN_NODE = ""
OBJ_PLAN_NODE = "#PLAN_NODE#/world_model"


FOLDER = "launchesShelf/"
if not os.path.isdir(FOLDER):
    os.makedirs(FOLDER)
else:
    shutil.rmtree(FOLDER)
    os.makedirs(FOLDER)

#ALGORITHMS = ["single_search", "rearrangement_birrt", "rearrangement_prm"]
# PRIMITIVES = ["mrs", "nmrs" , "pap"]
# TESTS = ["rss", "shelf", "table"]
ALGORITHMS = ["single_search", "rearrangement_prm"]
SMOOTHER = "rearrangement_path_planner"
PRIMITIVES = ["mrs", "nmrs", "pap"]
mcr_testing = False
mcr_transit = False
MULTIPLIERS = "[1.1, 1.2, 1.3, 1.4, 1.5, 1.8, 2]"
TESTS = ["shelf"]
Kshelf = [2,4,6,8,10]#,12,14,16,18,20]
Krss = [6,11,16]
Kgrid = [4,8,12]
Kbeer = [6]
N = 20
NMRS_MULTIPLIERS = [1.1,1.2,1.3,1.4,1.5]
NMRS_MULTIPLIERS_NAMES = ["1_1","1_2","1_3","1_4","1_5"]
NMRS_MULTIPLIERS = [1000]
NMRS_MULTIPLIERS_NAMES = ["1000"]
EXACT_METHOD = "true"
MULTI_START = "true"   

TEST_NAME = ""
FOR_TESTING = "false"
SIMULATE = FOR_TESTING
VISUALIZE = FOR_TESTING
PESISTENT = FOR_TESTING
GATHER_STATISTICS = "true"
INIT_SMOOTHING = "true"
EXTRAS = ""

if GATHER_STATISTICS == "true":        
    STATISTICS_FILE = "statistics_file: " + "#STATISTICS_FOLDER#/#PLAN_NODE#_statistics.txt"
    print(STATISTICS_FILE)
else:
    STATISTICS_FILE = ""

SIMULATION = "" #will have the text for the simulation if is active
VISUALIZATION = "" #will have the text for the visualization if is active
PLANNING = ""
MAPPING_LIST = ""  
STATISTICS_FOLDER = ""
SMOOTHING = ""     

BAXTER=""
BAXTER_ARM = "left"
BAXTER_SAFE_STATE = "[0.25684,-1.34394,0.42624,2.61800,2.84100,2.09400,-0.06762,0]"
BAXTER_VIDEO = ""
BAXTER_ROTATION = ""
BAXTER_TRANSLATION = "[0,0,0.785]"

OBJECT = ""
ENVIRONMENT = ""
DISTANCE_METRIC = ""
final_poses_filename = ""
sampling_poses_filename = ""
DIM = "0.055 , 0.03 , 0.12"
RAD = "0.02"  
HEIGHT = "0.14"
BIASING = "10"
TIME_LIMIT = "1800"
ShelfColors=["red", "green", "blue", "yellow", "pink", "grey", "black", "purple", "orange", "light_blue"]
RSSColors=["red1", "red2","red3","red4","red5","red6", "green1", "green2", "green3", "green4", "green5", "blue1", "blue2", "blue3", "blue4", "blue5"]
GRIDColors=["red1", "red2", "green1", "green2", "blue1", "blue2", "red3","red4", "green3", "green4", "blue3", "blue4", "red5", "red6", "green5", "green6", "blue5", "blue6"]
Colors = []

clear_dist = 0.14
all_poses = []
final_poses = []

def init_test(test):
    global BAXTER
    global OBJECT
    global ENVIRONMENT
    global DIM
    global HEIGHT
    global RAD
    global BAXTER_ROTATION
    global STATISTICS_FILE
    global TEST_NAME
    global final_poses_filename
    global sampling_poses_filename
    global Colors
    global clear_dist   
    global all_poses 
    global final_poses
    global K
    global mcr_testing

    TEST_NAME = test + "_"

    if test == "shelf":
        BAXTER = ""
        BAXTER_ROTATION = "[1.0,0,0,0,1.0,0,0,0,1]"
        ENVIRONMENT = "shelf"
        Colors = []
        Colors.extend(ShelfColors)
        sampling_poses_filename = "poses/shelf_all_poses.txt"
        if mcr_testing == True:
            if mcr_transit == True:
                # sampling_poses_filename = "poses/shelf_mcr_transit_all_poses.txt"
                final_poses_filename = "poses/shelf_mcr_transit_final_poses.txt"
            else:
                final_poses_filename = "poses/shelf_mcr_final_poses.txt"
        else:
            final_poses_filename = "poses/shelf_final_poses.txt"
        clear_dist = 0.1

        OBJECT = ""
        fobject = open("templates/cup.txt","r")
        for line in fobject:
            line=re.sub('#RAD#', RAD, line)
            line=re.sub('#HEIGHT#', HEIGHT, line)
            OBJECT = OBJECT + line    
        K = []
        K.extend(Kshelf)   
    elif test == "beer":
        BAXTER = ""
        BAXTER_ROTATION = "[1.0,0,0,0,1.0,0,0,0,1]"
        ENVIRONMENT = "beer_setup"
        Colors = ["black", "red", "green", "blue", "yellow", "grey"]
        K = [6]                
        final_poses_filename = "poses/beer_final_poses.txt"
        sampling_poses_filename = "poses/beer_start_poses.txt"  
        RAD = "0.02"  
        HEIGHT = "0.14"
        clear_dist = 0.0001

        fobject = open("templates/cup.txt","r")
        for line in fobject:
            line=re.sub('#RAD#', RAD, line)
            line=re.sub('#HEIGHT#', HEIGHT, line)
            OBJECT = OBJECT + line
    else:
        BAXTER = "_side"
        BAXTER_ROTATION = "[0,1.0,0,-1.0,0,0,0,0,1]"
        ENVIRONMENT = "rss_table"
        Colors = []
        K = []        
        if test == "table":
            final_poses_filename = "poses/grid_final_poses.txt"
            print(test)
            print(final_poses_filename)
            Colors.extend(GRIDColors)
            K.extend(Kgrid)
        else:
            final_poses_filename = "poses/rss_final_poses.txt"
            print(test)
            print(final_poses_filename)
            Colors.extend(RSSColors)
            K.extend(Krss)

        sampling_poses_filename = "poses/table_all_poses.txt"
        clear_dist = 0.14

        OBJECT = ""
        fobject = open("templates/box.txt","r")
        for line in fobject:    
            line=re.sub('#DIM#', DIM, line)
            OBJECT = OBJECT + line

    all_poses = []
    fin = open(sampling_poses_filename,"r")
    for line in fin.readlines():
        line = line.rstrip('\n')
        all_poses.append(line)
    fin.close()

    final_poses = []
    ffinal = open(final_poses_filename, "r")
    for line in ffinal.readlines():
        line = line.rstrip('\n')
        final_poses.append(line)
    ffinal.close()

def init_algo(algo):
    global DISTANCE_METRIC
    global MIN_CONFLICT
    global SHORTEST_PATH
    global EXACT_METHOD
    global MULTI_START
    global EXTRAS
    global INIT_SMOOTHING
    global SMOOTHING

    if algo == "cnmrs" or algo == "crs" or algo == "cfmrs":
        DISTANCE_METRIC = "type: graph_distance_metric\n          distance_function: poses_distance"
    else:
        DISTANCE_METRIC = "type: linear_distance_metric"

    if algo == "mrs":
        MIN_CONFLICT = "false"
        SHORTEST_PATH = "true"
        SMOOTHING = "false"
    else:
        MIN_CONFLICT = "true"
        SHORTEST_PATH = "false"
        SMOOTHING = INIT_SMOOTHING

    if "fmrs" in algo:
        print(algo)
        if(algo == "fmrsexact"):
            EXTRAS = "exact_method: true\n        multi_start: false"
        elif(algo == "fmrsgreedy"):
            EXTRAS = "exact_method: false\n        multi_start: false"
        elif(algo == "fmrsmulti"):
            EXTRAS = "exact_method: false\n        multi_start: true"            
        else:
            EXTRAS = "exact_method: true\n        multi_start: true"

        if algo != "cfmrs":
            algo = "fmrs"
    elif algo == "mcr_test":        
        EXTRAS = "multipliers: " + MULTIPLIERS
        if mcr_transit == True:
            EXTRAS = EXTRAS  + "\n        transit_mode: true"
    else:
        EXTRAS = ""


    return algo                        


def calc_dist(p1,p2):
    return math.sqrt(math.pow((p2[0] - p1[0]), 2) +
                     math.pow((p2[1] - p1[1]), 2) +
                     math.pow((p2[2] - p1[2]), 2))

def collision_free_pose(p):
    global clear_dist
    global used_poses

    p_split = p.split(',') 
    p_point = (float(p_split[0]), float(p_split[1]), float(p_split[2])) 
    for pose in used_poses:
        if calc_dist(p_point,pose)  < clear_dist:
            return False

    used_poses.append(p_point)
    return True


def build_object(i, p):
    global plan_objects

    objP = OBJECT
    objP=re.sub('#NODE#', OBJ_PLAN_NODE, objP)
    objP=re.sub('#NAME#', "cup", objP)
    objP=re.sub('#ID#', str(i+1), objP)
    objP=re.sub('#INITIAL_STATE#', "", objP)
    objP=re.sub('#COLOR#', Colors[0], objP)

    plan_objects = plan_objects + objP + "\n\n"


def build_problem(k,t):
    global poses
    global s_poses
    global f_poses
    global objects
    global plan_objects
    global used_poses
    global mcr_transit

    poses = []       
    poses.extend(all_poses)    
    used_poses = []
    s_poses = ""
    f_poses = ""
    objects = ""
    plan_objects = ""

    if mcr_transit == True:
        p = random.choice(final_poses)
        s_poses = s_poses + "          -\n" + "            pose: [" + p + "]\n" 
        p = random.choice(poses)
        f_poses = f_poses + "          -\n" + "            pose: [" + p + "]\n"
        build_object(0,p)
        for i in range(1,k):  
            p = random.choice(poses)
            while(collision_free_pose(p) == False):
                poses.remove(p)
                if len(poses) == 0:
                    return False
                p = random.choice(poses)
            s_poses = s_poses + "          -\n" + "            pose: [" + p + "]\n" 
            f_poses = f_poses + "          -\n" + "            pose: [" + p + "]\n"
            build_object(i,p)
            if p in poses:
                poses.remove(p)
            if len(poses) == 0:
                return False
        # for i in range(k-1):            
        #     if i < 9:
        #         p = poses[0]
        #         s_poses = s_poses + "          -\n" + "            pose: [" + p + "]\n" 
        #         f_poses = f_poses + "          -\n" + "            pose: [" + p + "]\n"
        #         build_object(i,p)
        #         if p in poses:
        #             poses.remove(p)
        #     else:
        #         p = random.choice(poses)
        #         while(collision_free_pose(p) == False):
        #             poses.remove(p)
        #             if len(poses) == 0:
        #                 return False
        #             p = random.choice(poses)
        #         s_poses = s_poses + "          -\n" + "            pose: [" + p + "]\n" 
        #         if i == k-1:
        #             p = final_poses[0];
        #         f_poses = f_poses + "          -\n" + "            pose: [" + p + "]\n"
        #         build_object(i,p)
        #         if p in poses:
        #             poses.remove(p)
        #         if len(poses) == 0:
        #             return False
    else:
        for i in range(k):
            p = random.choice(poses)
            while(collision_free_pose(p) == False):
                poses.remove(p)
                if len(poses) == 0:
                    return False
                p = random.choice(poses)
            s_poses = s_poses + "          -\n" + "            pose: [" + p + "]\n" 
            if t == "beer":
                if i == 0:
                    f_poses = f_poses + "          -\n" + "            pose: [" + final_poses[i] + "]\n"
                else:
                    f_poses = f_poses + "          -\n" + "            pose: [" + p + "]\n"
            else:
                f_poses = f_poses + "          -\n" + "            pose: [" + final_poses[i] + "]\n"
            build_object(i,p)
            poses.remove(p)
            if len(poses) == 0:
                return False
    return True

K = []
poses = []
used_poses = []
s_poses = ""
f_poses = ""
plan_objects = ""

allname = open("all_names.txt","w")
allname2 = open(FOLDER + "all_names.txt","w")
fall = open(FOLDER + "all_launches.launch","w")
fall.write("<launch>")

for search in ALGORITHMS:
    for a in PRIMITIVES:
        algo_filename = FOLDER + search + "_" + a + ".launch"
        falg = open(algo_filename,"w")
        falg.write("<launch>")
        falg.close()

for t in TESTS:
    init_test(t)    
    TEST_NAME = t + "_"
    for k in K:
        for n in range(N):

            n_str = str(n);
            if n < 10:
                n_str = "0"+n_str;        

            while build_problem(k,t) == False:
                print("Failed to build the problem: " + t + "_" + str(k) + "_" + n_str)
            
            first_time = True;                        
            for search in ALGORITHMS:
                for a in PRIMITIVES:
                    if a == "pap" and first_time == False:
                        continue;
                    elif a == "pap":
                        first_time = False;
                        search = "rearrangement_prm"

                    if a == "nmrs":
                        for multi_index in xrange(len(NMRS_MULTIPLIERS)):
                            print(multi_index)
                            combine  = str(search) + "_" + str(a) + "_" + NMRS_MULTIPLIERS_NAMES[multi_index] + "_" + str(t) + "_"+ str(k) + "_" + n_str
                            PLAN_NODE = "planning_" + combine
                            final_statistics_file = re.sub('#PLAN_NODE#', PLAN_NODE, STATISTICS_FILE)
                            final_statistics_file = re.sub('#STATISTICS_FOLDER#', search +"_"+ a, final_statistics_file)
                            allname.write(PLAN_NODE)
                            allname.write("\n");
                            allname2.write(PLAN_NODE)
                            allname2.write("\n");
                            a = init_algo(a)
                            fin = open("templates/run_template.txt", "r")
                            
                            filename = FOLDER +combine+".launch"
                            fout = open(filename, "w");

                            algo_filename = FOLDER + search + "_" + a + ".launch"
                            falg = open(algo_filename,"a")
                            falg.write("\n<include file=\"$(find labeled_rearrangement)/input/" + filename + "\"/>")
                            falg.close()

                            fall.write("\n<include file=\"$(find labeled_rearrangement)/input/" + filename + "\"/>")

                            plannode_objects = re.sub('#PLAN_NODE#', PLAN_NODE, plan_objects)
                            EXTRAS = "length_multiplier: " + str(NMRS_MULTIPLIERS[multi_index])
                            for line in fin.readlines():
                                line=re.sub('#PACKAGE#', PACKAGE, line)                    
                                line=re.sub('#PLAN_NODE#', PLAN_NODE, line)
                                line=re.sub('#TEST#', t, line)
                                line=re.sub('#TEST_NAME#', TEST_NAME, line)
                                line=re.sub('#ALGORITHM#', search, line)
                                line=re.sub('#PRIMITIVE#', a, line)
                                line=re.sub('#EXTRAS#', EXTRAS, line)
                                line=re.sub('#SMOOTHER#', SMOOTHER, line)   
                                line=re.sub('#SMOOTHING#', SMOOTHING, line)                 
                                line=re.sub('#BAXTER#', BAXTER, line)
                                line=re.sub('#BAXTER_VIDEO#', BAXTER_VIDEO, line)                    
                                line=re.sub('#BAXTER_ARM#', BAXTER_ARM, line)                    
                                line=re.sub('#BAXTER_ROTATION#', BAXTER_ROTATION, line)
                                line=re.sub('#BAXTER_TRANSLATION#', BAXTER_TRANSLATION, line)                    
                                line=re.sub('#BAXTER_SAFE_STATE#', BAXTER_SAFE_STATE, line)
                                line=re.sub('#ENVIRONMENT#', ENVIRONMENT, line)
                                line=re.sub('#SIMULATE#', SIMULATE, line)
                                line=re.sub('#VISUALIZE#', VISUALIZE, line)
                                line=re.sub('#PESISTENT#', PESISTENT, line)                    
                                line=re.sub('#DISTANCE_METRIC#', DISTANCE_METRIC, line)                    
                                line=re.sub('#Kval#', str(k), line)
                                line=re.sub('#SPOSES#', s_poses, line)
                                line=re.sub('#FPOSES#', f_poses, line)
                                line=re.sub('#BIASING#', BIASING, line)
                                line=re.sub('#TIME_LIMIT#', TIME_LIMIT, line)

                                line=re.sub('#MIN_CONFLICT#', MIN_CONFLICT, line)
                                line=re.sub('#SHORTEST_PATH#', SHORTEST_PATH, line)
                                
                                line=re.sub('#WORLD_MODEL#', plannode_objects, line)                    
                                line=re.sub('#GATHER_STATISTICS#', GATHER_STATISTICS, line)
                                line=re.sub('#STATISTICS_FILE#', final_statistics_file, line)

                                line=re.sub('#OBJECTS#', "" , line)
                                line=re.sub('#SIMULATION#', "" , line)
                                line=re.sub('#VISUALIZATION#', "" , line)        
                                line=re.sub('#PLANNING#', "" , line)
                                fout.write(line)                   
                            fout.close()                
                            fin.close()
                    else:
                        combine  = str(search) + "_" + str(a) + "_" + str(t) + "_"+ str(k) + "_" + n_str
                        PLAN_NODE = "planning_" + combine
                        final_statistics_file = re.sub('#PLAN_NODE#', PLAN_NODE, STATISTICS_FILE)
                        final_statistics_file = re.sub('#STATISTICS_FOLDER#', search +"_"+ a, final_statistics_file)
                        allname.write(PLAN_NODE)
                        allname.write("\n");
                        allname2.write(PLAN_NODE)
                        allname2.write("\n");
                        a = init_algo(a)
                        fin = open("templates/run_template.txt", "r")
                        
                        filename = FOLDER +combine+".launch"
                        fout = open(filename, "w");

                        algo_filename = FOLDER + search + "_" + a + ".launch"
                        falg = open(algo_filename,"a")
                        falg.write("\n<include file=\"$(find labeled_rearrangement)/input/" + filename + "\"/>")
                        falg.close()

                        fall.write("\n<include file=\"$(find labeled_rearrangement)/input/" + filename + "\"/>")

                        plannode_objects = re.sub('#PLAN_NODE#', PLAN_NODE, plan_objects)
                        for line in fin.readlines():
                            line=re.sub('#PACKAGE#', PACKAGE, line)                    
                            line=re.sub('#PLAN_NODE#', PLAN_NODE, line)
                            line=re.sub('#TEST#', t, line)
                            line=re.sub('#TEST_NAME#', TEST_NAME, line)
                            line=re.sub('#ALGORITHM#', search, line)
                            line=re.sub('#PRIMITIVE#', a, line)
                            line=re.sub('#EXTRAS#', EXTRAS, line)
                            line=re.sub('#SMOOTHER#', SMOOTHER, line)   
                            line=re.sub('#SMOOTHING#', SMOOTHING, line)                 
                            line=re.sub('#BAXTER#', BAXTER, line)
                            line=re.sub('#BAXTER_VIDEO#', BAXTER_VIDEO, line)                    
                            line=re.sub('#BAXTER_ARM#', BAXTER_ARM, line)                    
                            line=re.sub('#BAXTER_ROTATION#', BAXTER_ROTATION, line)
                            line=re.sub('#BAXTER_TRANSLATION#', BAXTER_TRANSLATION, line)                    
                            line=re.sub('#BAXTER_SAFE_STATE#', BAXTER_SAFE_STATE, line)
                            line=re.sub('#ENVIRONMENT#', ENVIRONMENT, line)
                            line=re.sub('#SIMULATE#', SIMULATE, line)
                            line=re.sub('#VISUALIZE#', VISUALIZE, line)
                            line=re.sub('#PESISTENT#', PESISTENT, line)                    
                            line=re.sub('#DISTANCE_METRIC#', DISTANCE_METRIC, line)                    
                            line=re.sub('#Kval#', str(k), line)
                            line=re.sub('#SPOSES#', s_poses, line)
                            line=re.sub('#FPOSES#', f_poses, line)
                            line=re.sub('#BIASING#', BIASING, line)
                            line=re.sub('#TIME_LIMIT#', TIME_LIMIT, line)

                            line=re.sub('#MIN_CONFLICT#', MIN_CONFLICT, line)
                            line=re.sub('#SHORTEST_PATH#', SHORTEST_PATH, line)
                            
                            line=re.sub('#WORLD_MODEL#', plannode_objects, line)                    
                            line=re.sub('#GATHER_STATISTICS#', GATHER_STATISTICS, line)
                            line=re.sub('#STATISTICS_FILE#', final_statistics_file, line)

                            line=re.sub('#OBJECTS#', "" , line)
                            line=re.sub('#SIMULATION#', "" , line)
                            line=re.sub('#VISUALIZATION#', "" , line)        
                            line=re.sub('#PLANNING#', "" , line)
                            fout.write(line)                   
                        fout.close()                
                        fin.close()

for search in ALGORITHMS:
    for a in PRIMITIVES:
        algo_filename = FOLDER + search + "_" + a + ".launch"
        falg = open(algo_filename,"a")
        falg.write("\n</launch>")
        falg.close()

fall.write("\n</launch>")
fall.close()    
allname.close()
allname2.close()