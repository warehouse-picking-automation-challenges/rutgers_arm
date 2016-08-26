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
	for RN in xrange(1,11)
		fin=open(filename,"r")
		fout=open("balls_"+str(n)+"_"+str(RN)+".yaml","w")
		ball=0
		for line in fin.readlines():
			x=0
			y=0
			z=0
			position=""
			if random.randint(0,1) == 0:
				z=random.uniform(3.7,5)
				y=random.uniform(0.4,1.35)
				x=random.uniform(0.4,0.95)
				position=str(x)+", "+str(y)+", "+str(z)
			else:
				while True:
					z=random.uniform(4.1,5)
					y=random.uniform(0,1.35)
					x=random.uniform(0,0.95)
					position=str(x)+", "+str(y)+", "+str(z)
					if x > 0.3 and y > 0.3:
						break
			print(position)
			line=re.sub('#POS#', position, line)
			line=re.sub('#RAD#', str(radius), line)
			fout.write(line)