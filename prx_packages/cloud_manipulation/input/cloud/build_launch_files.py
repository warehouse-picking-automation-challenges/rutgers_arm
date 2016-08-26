import re
from array import *
from math import *
import random
print("Python !")
RANDOM=143918341
EN=1
total_launch=open("launch_all.launch", "w")
total_launch.write("<launch>\n")
allnames=open("all_names.txt","w")
path=""
for RM in [5000]:
	for RN in xrange(3,6):
		#for PL in ['prm_star','irs','spars']:
		for PL in ['prm_star']:
			if PL=='prm_star':
				path="$(find prx_input)/templates/motion_planners/prm_star.yaml"
			if PL=='irs':
				path="$(find fast_IRS)/input/firs.yaml"
			if PL=='spars':
				path="$(find SPARS)/input/spars.yaml"		
			#print('RM='+str(RM)+', RN='+str(RN)+', PL='+str(PL))
			filename="_"+str(RM)+"RM_"+str(RN)+"RN_"+str(EN)+"EN_"+str(PL)+"PL"
			print("planning"+filename)
			allnames.write("planning"+filename+"\n")
			fout=open("cloud"+filename+".launch", "w")
			fin=open("launch_template.txt", "r")
			for line in fin.readlines():
				line=re.sub('#RN#', str(RN), line)
				line=re.sub('#RM#', str(RM), line)
				line=re.sub('#EN#', str(EN), line)
				line=re.sub('#PL#', str(PL), line)
				line=re.sub('#PATH#', path, line)
				fout.write(line)
				#print(line)
			total_launch.write("<include file=\"$(find cloud_manipulation)/input/cloud/cloud"+filename+".launch\"/>\n")
total_launch.write("\n</launch>")