import re
from array import *
from math import *
import random
print("Python !")
for k in xrange(2,9):
        for b in xrange(1,k+3):
                for n in xrange(1,11):
                        print('k='+str(k)+', b='+str(b)+', n='+str(n))
                        fout=open("wafr_"+str(k)+"k_"+str(b)+"b_"+str(n)+"n.launch", "w")
                        fin=open("template_wafr.launch", "r")
                        i=0
                        for line in fin.readlines():
                                line=re.sub('##B##', str(b), line)
                                line=re.sub('##K##', str(k), line)
                                line=re.sub('##N##', str(n), line)
                                fout.write(line)
                                #print(line)