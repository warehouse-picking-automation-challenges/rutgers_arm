import re
from array import *
from math import *
import random
import math

PACKAGE = "labeled_rearrangement"
SIM_NODE = "simulation"
PLAN_NODE = "planning"
OBJ_PLAN_NODE = PLAN_NODE + "/world_model"
ROBOT = "motoman"

EXTRAS = ""

FOLDER = "launches/"
test = "rss"
TEST_NAME = test + "_"
# ALGORITHM = ["single_search", "rearrangement_birrt", "rearrangement_prm"]
#ALGORITHM = "mcr"
#PRIMITIVE = "mcr_test"
SMOOTHER = "rearrangement_path_planner"
ALGORITHM = "single_search"
PRIMITIVE = "nmrs"

EXTRAS = "length_multiplier: 100"


K = [8]
N = 1

FOR_TESTING = "true"
SIMULATE = FOR_TESTING
VISUALIZE = FOR_TESTING
PESISTENT = FOR_TESTING
GATHER_STATISTICS = "true"
SMOOTHING = "true"


if GATHER_STATISTICS == "true":        
    STATISTICS_FILE = "statistics_file: " + PRIMITIVE + "/" +PLAN_NODE+"_statistics.txt"
    print(STATISTICS_FILE)
else:
    STATISTICS_FILE = ""

SIMULATION = "" #ill have the text for the simulation if is active
VISUALIZATION = "" #will have the text for the visualization if is active
PLANNING = ""
MAPPING_LIST = ""        
OBJECT = ""
DISTANCE_METRIC = ""
BIASING = "10"
TIME_LIMIT = "90"

BAXTER_VIDEO = "_nomesh"
BAXTER_ARM = "left"
SAFE_STATE = "[0.25684,-1.34394,0.42624,2.61800,2.84100,2.09400,-0.06762,0]"
MOTOMAN_SAFE_STATE = "[0 1.57 0 0 -1.70 0 0 0 1.57 0 0 -1.70 0 0 0 0 0]"


if ROBOT == "motoman":
    SAFE_STATE = MOTOMAN_SAFE_STATE


if "fmrs" in PRIMITIVE:
    EXTRAS = "exact_method: true\n        multi_start: true"
elif "mcr" in PRIMITIVE:
    EXTRAS = "multipliers: [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 2 3 4 5]"
    

Colors=["red", "green", "blue", "yellow", "pink", "grey", "black", "purple", "orange", "light_blue"]
RSSColors=["red1", "red2","red3","red4","red5","red6", "green1", "green2", "green3", "green4", "green5", "blue1", "blue2", "blue3", "blue4", "blue5"]
GRIDColors=["red1", "red2", "green1", "green2", "blue1", "blue2", "red3","red4", "green3", "green4", "blue3", "blue4", "red5", "red6", "green5", "green6", "blue5", "blue6"]
SoCSColors=["blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue","blue"]
SoCSColors=["red", "green", "blue", "yellow", "pink", "grey", "black", "purple", "orange", "light_blue"]
B = [0]

clear_dist = 0.14
BAXTER = ""
if ROBOT == "motoman":
    ENVIRONMENT = "rss_table_setup"
    sampling_poses_filename = "poses/motoman_rss_all_poses.txt"
    final_poses_filename = "poses/motoman_rss_final_poses.txt"
    DIM = "0.055 , 0.03 , 0.12" 
    clear_dist = 0.05 

    fobject = open("templates/box.txt","r")
    for line in fobject:    
        line=re.sub('#DIM#', DIM, line)
        OBJECT = OBJECT + line
elif test == "shelf":
    BAXTER = ""
    ENVIRONMENT = "shelf"
    final_poses_filename = "poses/shelf_final_poses.txt"
    sampling_poses_filename = "poses/shelf_all_poses.txt"
    RAD = "0.02"  
    HEIGHT = "0.14"

    fobject = open("templates/cup.txt","r")
    for line in fobject:
        line=re.sub('#RAD#', RAD, line)
        line=re.sub('#HEIGHT#', HEIGHT, line)
        OBJECT = OBJECT + line
    if ALGORITHM == "mcr":
        Colors = SoCSColors;
elif test == "hanoi":
    BAXTER = ""
    BAXTER_ROTATION = "[1.0,0,0,0,1.0,0,0,0,1]"
    ENVIRONMENT = "hanoi_kiva_pod"
    Colors = ["red", "green", "blue"]
    K = [3]                
    final_poses_filename = "poses/hanoi_final_poses.txt"
    sampling_poses_filename = "poses/hanoi_start_poses.txt"  
    RAD = "0.02"  
    HEIGHT = "0.14"
    clear_dist = 0.0001

    fobject = open("templates/cup.txt","r")
    for line in fobject:
        line=re.sub('#RAD#', RAD, line)
        line=re.sub('#HEIGHT#', HEIGHT, line)
        OBJECT = OBJECT + line
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
    ENVIRONMENT = "rss_table"    
    # sampling_poses_filename = "poses/grid_final_poses.txt"
    sampling_poses_filename = "poses/rss_all_poses.txt"
    DIM = "0.055 , 0.03 , 0.12"  

    if test == "table":
        final_poses_filename = "poses/grid_final_poses.txt"
    else:
        final_poses_filename = "poses/rss_final_poses.txt"

    # final_poses_filename = "poses/rss_all_poses.txt"
    fobject = open("templates/box.txt","r")
    for line in fobject:    
        line=re.sub('#DIM#', DIM, line)
        OBJECT = OBJECT + line


if BAXTER == "_side" :
    BAXTER_ROTATION = "[0,1.0,0,-1.0,0,0,0,0,1]"
    BAXTER_TRANSLATION = "[0,0,0.785]"
else:
    BAXTER_ROTATION = "[1.0,0,0,0,1.0,0,0,0,1]"
    BAXTER_TRANSLATION = "[0,0,0.785]"

if PRIMITIVE == "cnmrs" or PRIMITIVE == "crs":
    DISTANCE_METRIC = "type: graph_distance_metric\n          distance_function: poses_distance"
else:
    DISTANCE_METRIC = "type: linear_distance_metric"

if PRIMITIVE == "mrs":
    MIN_CONFLICT = "false"
    SHORTEST_PATH = "true"
else:
    MIN_CONFLICT = "true"
    SHORTEST_PATH = "false"

STATISTICS_FILE = ""
if GATHER_STATISTICS == "true":
    STATISTICS_FILE = "statistics_file: " + test +"_"+ PLAN_NODE + "_statistics.txt"

if SIMULATE == "true":
    if ROBOT == "motoman":
        fsim = open("templates/common_motoman_simulation.txt","r")

        for line in fsim.readlines():
            line=re.sub('#PACKAGE#', PACKAGE, line)
            line=re.sub('#SAFE_STATE#', SAFE_STATE, line)
            line=re.sub('#ENVIRONMENT#', ENVIRONMENT, line)
            SIMULATION = SIMULATION + line
    else:
        fsim = open("templates/common_simulation.txt","r")
        
        for line in fsim.readlines():
            line=re.sub('#PACKAGE#', PACKAGE, line)
            line=re.sub('#BAXTER#', BAXTER, line)
            line=re.sub('#BAXTER_VIDEO#', BAXTER_VIDEO, line)
            line=re.sub('#ENVIRONMENT#', ENVIRONMENT, line)
            line=re.sub('#BAXTER_ROTATION#', BAXTER_ROTATION, line)
            line=re.sub('#BAXTER_TRANSLATION#', BAXTER_TRANSLATION, line)
            line=re.sub('#SAFE_STATE#', SAFE_STATE, line)
            SIMULATION = SIMULATION + line

if VISUALIZE == "true":    
    if test == "rss" or test == "table":
        fvis = open("templates/rss_visualization.txt","r")
    elif test == "shelf":
        fvis = open("templates/shelf_visualization.txt","r")
    else:
        fvis = open("templates/visualization.txt","r")

    for line in fvis.readlines():
        VISUALIZATION = VISUALIZATION + line

if FOR_TESTING == "true":
    PLANNING = "<node name=\"planning\" pkg=\"prx_planning\" type=\"prx_planning\" required=\"true\" launch-prefix=\"gdb --args\" output=\"screen\" args=\"" + PLAN_NODE + "\"/>"


fin_final_poses = open(final_poses_filename,"r")
fin_poses = open (sampling_poses_filename,"r")
all_poses = []
final_poses = []
poses = []
used_poses = []
s_poses = ""
f_poses = ""
objects = ""
plan_objects = ""
object_pairs = ""


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
    global objects
    global plan_objects
    global MAPPING_LIST
    
    if(SIMULATE == "true"):
        obj = OBJECT
        obj=re.sub('#NODE#', SIM_NODE, obj)
        obj=re.sub('#NAME#', "cup", obj)
        obj=re.sub('#ID#', str(i+1), obj)
        obj=re.sub('#INITIAL_STATE#', "initial_state: [" + p + "]", obj)
        if test == "rss":
            obj=re.sub('#COLOR#', RSSColors[i], obj)
        elif test == "table":
            obj=re.sub('#COLOR#', GRIDColors[i], obj)
        else:
            obj=re.sub('#COLOR#', Colors[i], obj)
        objects = objects + obj + "\n\n"

        MAPPING_LIST = MAPPING_LIST + "      -\n        pair: [simulator/cup" + str(i+1) + ", world_model/simulator/cup" + str(i+1) +"]\n"

    objP = OBJECT
    objP=re.sub('#NODE#', OBJ_PLAN_NODE, objP)
    objP=re.sub('#NAME#', "cup", objP)
    objP=re.sub('#ID#', str(i+1), objP)
    objP=re.sub('#INITIAL_STATE#', "", objP)
    if test == "rss":
        objP=re.sub('#COLOR#', RSSColors[i], objP)
    else:
        objP=re.sub('#COLOR#', Colors[i], objP)

    
    plan_objects = plan_objects + objP + "\n\n"

def build_problem(k):
    global test
    global poses
    global s_poses
    global f_poses
    global objects
    global plan_objects
    global used_poses
    global MAPPING_LIST

    poses = []    
    poses.extend(all_poses)
    used_poses = []
    s_poses = ""
    f_poses = ""
    objects = ""
    plan_objects = ""
    MAPPING_LIST = ""

    if(test == "hanoi"):
        for i in range(k):
            p = poses[i]
            s_poses = s_poses + "          -\n" + "            pose: [" + p + "]\n" 
            f_poses = f_poses + "          -\n" + "            pose: [" + final_poses[i] + "]\n"
            build_object(i,p)
        return True


    for i in range(k):
        print(str(i) + "/" + str(len(poses)))
        p = random.choice(poses)
        while(collision_free_pose(p) == False):
            poses.remove(p)
            if len(poses) == 0:
                return False
            p = random.choice(poses)
            
        s_poses = s_poses + "          -\n" + "            pose: [" + p + "]\n" 
        if test == "beer":
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

    print(str(i) +  "  TRUE  for k : " + str(k))
    return True

for line in fin_poses.readlines():
    line = line.rstrip('\n')
    all_poses.append(line)

for line in fin_final_poses.readlines():
    line = line.rstrip('\n')
    final_poses.append(line)

for k in K:
    for n in range(N):                
        while build_problem(k) == False:
            print("Could not find problem for : " + PRIMITIVE + "_" + test + "_" + str(k))
        if ROBOT == "motoman":
            fin = open("templates/run_motoman_template.txt", "r")
        else:
            fin = open("templates/run_template.txt", "r")
        n_str = str(n);
        if n < 10:
            n_str = "0"+n_str;
        filename = "launches/"+test+"_"+PRIMITIVE+"_"+str(k)+"k_"+n_str+".launch"
        fout = open(filename, "w");
        SIMULATION=re.sub('#MAPPING_LIST#', MAPPING_LIST, SIMULATION)
        for line in fin.readlines():
            line=re.sub('#PACKAGE#', PACKAGE, line)                    
            line=re.sub('#PLAN_NODE#', PLAN_NODE, line)
            line=re.sub('#TEST#', test, line)
            line=re.sub('#TEST_NAME#', TEST_NAME, line)
            line=re.sub('#ALGORITHM#', ALGORITHM, line)
            line=re.sub('#PRIMITIVE#', PRIMITIVE, line)
            line=re.sub('#EXTRAS#', EXTRAS, line)
            line=re.sub('#SMOOTHER#', SMOOTHER, line)   
            line=re.sub('#SMOOTHING#', SMOOTHING, line)                 
            line=re.sub('#BAXTER#', BAXTER, line)
            line=re.sub('#BAXTER_VIDEO#', BAXTER_VIDEO, line)                    
            line=re.sub('#BAXTER_ARM#', BAXTER_ARM, line)                    
            line=re.sub('#BAXTER_ROTATION#', BAXTER_ROTATION, line)
            line=re.sub('#BAXTER_TRANSLATION#', BAXTER_TRANSLATION, line)                                   
            line=re.sub('#SAFE_STATE#', SAFE_STATE, line)
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
            
            line=re.sub('#WORLD_MODEL#', plan_objects, line)                    
            line=re.sub('#GATHER_STATISTICS#', GATHER_STATISTICS, line)
            line=re.sub('#STATISTICS_FILE#', STATISTICS_FILE, line)

            line=re.sub('#OBJECTS#', objects, line)
            line=re.sub('#SIMULATION#', SIMULATION, line)
            line=re.sub('#VISUALIZATION#', VISUALIZATION, line)        
            line=re.sub('#PLANNING#', PLANNING, line)   
            fout.write(line)
        fin.close()

print("DONE")
