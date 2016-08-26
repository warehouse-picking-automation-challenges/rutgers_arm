import sys
import random
import math

random.seed(123);

f = open("params.yaml",'w');
f.write("type: bullet_plant\ngeometries:\n")

for i in xrange(500):
	x = random.uniform(-9,9)
	y = random.uniform(-9,9)
	rad = random.uniform(.1,.4)
	f.write("  -\n")
	f.write("    name: ball"+str(i)+"\n")
	f.write("    collision_geometry:\n")
	f.write("      type: sphere\n")
	f.write("      radius: "+str(rad)+"\n")
	f.write("      material: brown\n")
	f.write("    config:\n")
	f.write("      position: ["+str(x)+" "+str(y)+" 5]\n")
	f.write("      orientation: [0,0,0,1]\n")
	f.write("    mass: 1.0\n")
