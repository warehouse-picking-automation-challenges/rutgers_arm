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
for RM in [30000]:
	for RN in xrange(11,21):
		#for PL in ['prm_star','irs','spars']:
		for PL in ['_prm_star']:
			if PL=='_prm_star':
				path="$(find prx_input)/templates/motion_planners/prm_star.yaml"
			if PL=='_irs':
				path="$(find fast_IRS)/input/firs.yaml"
			if PL=='_spars':
				path="$(find SPARS)/input/spars.yaml"		
			#print('RM='+str(RM)+', RN='+str(RN)+', PL='+str(PL))
			filename="_"+str(RM)+"RM_"+str(RN)+"RN_"+str(EN)+"EN_"+str(PL)+"PL"
			print("planning"+filename)
			allnames.write("planning"+filename+"\n")
			fout=open("cloud_ram"+filename+".launch", "w")
			fin=open("launch_template_generation_ram.txt", "r")
			for line in fin.readlines():
				line=re.sub('#RN#', str(RN), line)
				line=re.sub('#RM#', str(RM), line)
				line=re.sub('#EN#', str(EN), line)
				line=re.sub('#PL#', str(PL), line)
				line=re.sub('#PATH#', path, line)
				fout.write(line)
				#print(line)
			total_launch.write("<include file=\"$(find cloud_manipulation)/input/cloud/cloud_ram"+filename+".launch\"/>\n")
total_launch.write("\n</launch>")