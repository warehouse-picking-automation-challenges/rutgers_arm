import os
import subprocess 
from time import sleep
for x in xrange(50):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "GTA_GRID_AVG/4_experiment/launches/main_example" + str(x) +".launch"])
	sleep(5)
for x in xrange(50):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "GTA_GRID_AVG/8_experiment/launches/main_example" + str(x) +".launch"])
	sleep(10)
for x in xrange(50):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "GTA_GRID_AVG/16_experiment/launches/main_example" + str(x) +".launch"])
	sleep(15)

for x in xrange(50):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "GTA_GRID_WORST/4_experiment/launches/main_example" + str(x) +".launch"])
	sleep(5)
for x in xrange(50):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "GTA_GRID_WORST/8_experiment/launches/main_example" + str(x) +".launch"])
	sleep(10)
for x in xrange(50):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "GTA_GRID_WORST/16_experiment/launches/main_example" + str(x) +".launch"])
	sleep(15)

for x in xrange(50):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "RVO_GRID/4_experiment/launches/main_example" + str(x) +".launch"])
	sleep(5)
for x in xrange(50):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "RVO_GRID/8_experiment/launches/main_example" + str(x) +".launch"])
	sleep(10)
for x in xrange(50):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "RVO_GRID/16_experiment/launches/main_example" + str(x) +".launch"])
	sleep(15)