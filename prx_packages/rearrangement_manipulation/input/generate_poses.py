import re
from array import *
from math import *
import random
import math
from subprocess import call

NODE = "planning"
PACKAGE = "rearrangement_manipulation"
SYSTEM = "baxter"
ENVIRONMENTS = ["table", "shelf"]
OBJECTS = ["crayola", "cup"]
NR_POSES = "2"
IK_SAMPLES = "2000"
OBJECT_SETUP = ""
OBJECT_USE = ""
OBJECT_ORIENTATION = ""

if SYSTEM == "baxter":
	START_LINK = "base"
	END_LINK = "end_effector_left"
	GRASP_FOLDER = "baxter_parallel_gripper"
else:
	START_LINK = "base_link"
	END_LINK = "head_sponge"
	GRASP_FOLDER = "motoman_unigripper"

obj_fin = open("templates/object.template")
object_template = obj_fin.read()

counter = -10;
for obj in OBJECTS:
	obj_type = obj + "_object"
	OBJECT_SETUP = OBJECT_SETUP + "  <rosparam command=\"load\" ns=\""+ NODE +"/" + obj_type + "\" file=\"$(find manipulation)/input/objects/" + obj +".yaml\"/>\n"

	obj_temp = object_template
	xyz = str(counter) + ", " + str(counter) + ", " + str(counter) 
	counter = counter - 10
	obj_temp=re.sub('#NODE#', NODE, obj_temp)
	obj_temp=re.sub('#OBJECT_NAME#', obj, obj_temp)
	obj_temp=re.sub('#OBJECT_TYPE#', obj_type , obj_temp)
	obj_temp=re.sub('#XYZ#', xyz, obj_temp)

	OBJECT_USE = OBJECT_USE + obj_temp + "\n"

	filename = "systems/" + obj + ".orientations"
	fin = open(filename, "r")
	OBJECT_ORIENTATION = OBJECT_ORIENTATION + fin.read() + "\n"


for ENVIRONMENT in ENVIRONMENTS:
	fin = open("templates/pose_generation.template", "r")

	filename = ENVIRONMENT+"_generate_poses.launch"
	print(filename)
	fout = open(filename, "w")

	for line in fin.readlines():
		line=re.sub('#NODE#', NODE, line)
		line=re.sub('#PACKAGE#', PACKAGE, line)
		line=re.sub('#SYSTEM#', SYSTEM, line)	    
		line=re.sub('#ENVIRONMENT#', ENVIRONMENT, line)
		line=re.sub('#NR_POSES#', NR_POSES, line)
		line=re.sub('#IK_SAMPLES#', IK_SAMPLES, line)
		line=re.sub('#START_LINK#', START_LINK, line)                    
		line=re.sub('#END_LINK#', END_LINK, line)	    
		line=re.sub('#GRASP_FOLDER#', GRASP_FOLDER, line)
		line=re.sub('#OBJECT_SETUP#', OBJECT_SETUP, line)
		line=re.sub('#OBJECT_USE#', OBJECT_USE, line)
		line=re.sub('#OBJECT_ORIENTATION#', OBJECT_ORIENTATION, line)
		fout.write(line)                   
	fout.close()                
	fin.close()

# FOR the objects
  # <rosparam command="load" ns="planning/cup_object" file="$(find manipulation)/input/objects/cup.yaml"/>
  # <rosparam command="load" ns="planning/crayola_object" file="$(find manipulation)/input/objects/crayola.yaml"/>

  #   <rosparam command="load" ns="planning/world_model/simulator/subsystems/box1">
  #   template: "cup_object"
  #   initial_state: [0.43270700,-0.02544752,0.86000000,0.60881596,-0.35964305,-0.60881596,-0.35964305]
  #   state_space:
  #     min: [0.3, -0.8, 0.86, -1, -1, -1, -1]
  #     max: [1.3, 0.8, 0.86, 1, 1, 1, 1]
  #     scale: [0.025, 0.025, 0.05, 0.5, 0.5, 0.5, 0.5]
  # </rosparam>