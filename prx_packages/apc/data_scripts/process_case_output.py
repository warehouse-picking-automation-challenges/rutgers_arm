import sys,os,re


columns = ["Plans","Grasps","Total","GraspSuccessTime","MotionSuccessTime","TotalSuccessTime","PathQuality","GraspFailureTime","MotionFailureTime","TotalFailureTime"]

data_points = {}

path =  os.getcwd()
dirs = os.listdir(path)

for filename in dirs:
	# filename="/home/zak/repos/pracsys_ws/src/pracsys/prx_output/case_stats.txt"
	if filename != "process_case_output"
		f = open(path+"/"+filename+"case_stats.txt",'r')
		lines = f.readlines()

		data_points[filename] = []

		f.close()
		for index in xrange(len(lines)):
			if "***CASEDATA" in lines[index]:
				data = re.split(": |\ |\n",lines[index+1])
				print data[1]+" "+data[3]+" "+data[5]
				if int(data[5]) < 2:
					data = re.split("/",lines[index+2])
					numbers =  [float(x) for x in data ]
					indexable = {}
					for i in xrange(len(columns)):
						indexable[columns[i]] = numbers[i]
					data_points[filename].append(indexable);
					# print indexable

print data_points

