import re
from array import *
from math import *
import random
print("Python !")
RANDOM=143918341
EN=1
total_launch=open("launch_all_resolve.launch", "w")
total_launch.write("<launch>\n")
path=""
all_names=open("all_names.txt","w")
#for RM in [500,1000,2000,5000,10000]:
for RM in [30000]:
	for RN in xrange(11,21):
		for PL in ['_prm_star']:#,'_spars','_irs']:#,'_spars','_irs']:
			#for PL in ['_prm_star']:
			if PL=='_prm_star':
				path="$(find prx_input)/templates/motion_planners/prm_star.yaml"
			if PL=='_irs':
				path="$(find fast_IRS)/input/firs.yaml"
			if PL=='_spars':
				path="$(find SPARS)/input/spars.yaml"		
			#print('RM='+str(RM)+', RN='+str(RN)+', PL='+str(PL))
			for n in ['hard']:
				for BN in xrange(2,3):
					filename="_"+str(RM)+"RM_"+str(RN)+"RN_"+str(EN)+"EN_"+str(PL)+"PL_"+str(n)+"_"+str(BN)
					print("planning"+filename+"\n")
					all_names.write("planning"+filename+"\n")
					fout=open("resolve"+filename+".launch", "w")
					fin=open("resolve_template.txt", "r")
					for line in fin.readlines():
						line=re.sub('#RN#', str(RN), line)
						line=re.sub('#RM#', str(RM), line)
						line=re.sub('#EN#', str(EN), line)
						line=re.sub('#PL#', str(PL), line)
						line=re.sub('#PATH#', path, line)
						line=re.sub('#BALL#', str(n), line)						
						line=re.sub('#BN#', str(BN), line)
						fout.write(line)
						#print(line)
					total_launch.write("<include file=\"$(find cloud_manipulation)/input/cloud/resolve_queries/resolve"+filename+".launch\"/>\n")
total_launch.write("\n</launch>")