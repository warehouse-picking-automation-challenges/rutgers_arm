import os
import subprocess 
from time import sleep
for x in xrange(10):
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "4agents_wait_rrvo/launches/main_example.launch"])
	sleep(10)
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "4agents_PWGreedy/launches/main_example.launch"])
	sleep(10)
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "4agents_PWGreedy_waiting/launches/main_example.launch"])
	sleep(10)
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "4agents_PWBest/launches/main_example.launch"])
	sleep(10)
	print "Next test " + str(x)
	subprocess.call(["roslaunch", "4agents_PWBest_waiting/launches/main_example.launch"])
	sleep(10)