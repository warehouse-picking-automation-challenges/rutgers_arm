import sys,os,re

path =  os.getcwd()
dirs = os.listdir(path)

from itertools import izip

def pairwise(iterable):
    a = iter(iterable)
    return izip(a, a)

for filename in dirs:
    # print filename
    if filename != "pracsys_format_reflex.py":
        g = open(filename,'r')
        data = g.readlines()
        g.close()
        g = open(filename,'w')
        for angles,line in pairwise(data):
            l = []

            a = re.split('\[|,|\]|\-',angles)
            for t in a:
                try:
                    l.append(float(t))
                except ValueError:
                    pass
            if l[0] < 1.55:
                new_data = line.replace("- [","-\n  relative_config: [")
                new_data = new_data.replace("]","]\n  release_mode: 3\n  grasp_mode: 2")
                g.write(new_data)
            else:
                new_data = line.replace("- [","-\n  relative_config: [")
                new_data = new_data.replace("]","]\n  release_mode: 4\n  grasp_mode: 5")           
                g.write(new_data)




# with open("vis_launch_params.launch",'r') as g:
#     data = g.readlines();