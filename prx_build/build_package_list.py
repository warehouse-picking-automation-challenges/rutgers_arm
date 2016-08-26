import os.path
import os
import sys
import shutil
import ast


def build_tuples():
    tuples = []
    pkgs = open('packages.config','r')
    for line in pkgs:        
        tuples.append(ast.literal_eval(line))
    return tuples

def get_tuple(filename,tuples):
    for t in tuples:
        if filename == t[0]:
            return t;
    return ('',-1);

def remove_unused(tuples,dirs):
    tuples = [tuples for x in tuples if x[0] in dirs];
    
def create_initial_file():
    dirs = [ name for name in os.listdir("../prx_packages") if os.path.isdir(os.path.join("../prx_packages", name)) ];
    pkgs = open('packages.config','w');
    for x in dirs:
        pkgs.write("('"+x+"',0)\n");

if not os.path.exists('packages.config'):
    create_initial_file();
else:
    pkgs = open('packages.config');
    pkgs_lines = pkgs.readlines();
    tuples = build_tuples();
    out = open('packages.config','w');
    dirs = [ name for name in os.listdir("../prx_packages") if os.path.isdir(os.path.join("../prx_packages", name)) ]
    remove_unused(tuples,dirs);
    for dir in dirs:
        t = get_tuple(dir,tuples);
        if t[0]=='':
            out.write("('"+dir+"',0)\n");
        else:
            out.write(str(t)+"\n");
            

