import os
from itertools import islice

ALGOS = ["mRS", "nmRS" , "CmRS" , "CnmRS" , "fmRS"]

folder = os.environ['PRACSYS_PATH'] + "/prx_output/rearrangement_graphs/Statistics/"
for alg in ALGOS:    
    files = folder + alg
    index = 0
    print(folder+alg + ".txt")    
    fout = open(folder+alg + ".txt", "a")
    for file in os.listdir(files):
        if file.endswith(".txt"):
            print(file)
            fin = open(files+"/"+file,"r")
            for line in islice(fin, index, 2):
                fout.write(line)
                index = 1
