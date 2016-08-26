import os
import subprocess 
from time import sleep
for x in xrange(1):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "2_experiment/launches/main_example.launch"])
	sleep(3)
for x in xrange(1):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "4_experiment/launches/main_example.launch"])
	sleep(5)
for x in xrange(1):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "8_experiment/launches/main_example.launch"])
	sleep(10)
for x in xrange(1):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "16_experiment/launches/main_example.launch"])
	sleep(15)
	