import re
from array import *
from math import *
import random
print("Python !")
EN=1
path=""
radius=0.05
for n in [2,4,6,8,16]:
	filename="balls."+str(n)+".yaml.template"
	for RN in xrange(1,11):
		fin=open(filename,"r")
		fout=open("balls_"+str(n)+"_"+str(RN)+".yaml","w")
		ball=0
		for line in fin.readlines():
			x=0
			y=0
			z=0
			position=""
			while True:
				if random.random() < 0.3:
					print("Close points")
					while True:
						z=random.uniform(3.8,4.1)
						y=random.uniform(-0.2,0.4)
						x=random.uniform(-0.2,0.4)
						position=str(x)+", "+str(y)+", "+str(z)
						if not(x < 0.2 and y < 0.2) and not(y<0.9 and x>-0.5 and x<0.23 and z<3.9) and not(y<0.4 and y>0.2 and x>0.6 and x<0.8 and z<3.9):
							break
				else:
					print("Random space")
					while True:
						z=random.uniform(3.5,4.1)
						y=random.uniform(-0.1,1.15)
						x=random.uniform(-0.15,0.8)
						position=str(x)+", "+str(y)+", "+str(z)
						if not(x<0.2 and y <0.2) and not(y<0.9 and x>-0.15 and x<0.15 and z<3.9) and not(y<0.4 and y>0.2 and x>0.6 and x<0.8 and z<3.9) and not(x>0.7 and y>0.9):
							break
				if not(x > 0.7 and y > 0.9):
					break
			print(position)
			line=re.sub('#POS#', position, line)
			line=re.sub('#RAD#', str(radius), line)
			fout.write(line)