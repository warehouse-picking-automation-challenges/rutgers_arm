import os
import subprocess 
from time import sleep
fin = open("batch_nodes.txt", "r")

N = 8

for line in fin.readlines():
    line = line.rstrip('\n')
    print "Current planning node "  + line
    for x in range(N):
        print "Next test " + str(x)
        subprocess.call(["rosrun", "prx_planning", "prx_planning", line])
        sleep(3)