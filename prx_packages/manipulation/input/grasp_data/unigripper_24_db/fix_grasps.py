import fnmatch
import os
import sys
#Create a folder for unigripper_24_db_fixed in the parent directory level and run the script
#in a directory that contains grasp database files that match the pattern submitted in the 
#obj name
def fix_grasps(obj_name, x_offset, y_offset, z_offset):
    for file in os.listdir('.'):
        if fnmatch.fnmatch(file, obj_name+'*.yaml'):
            print file
            f = open(file, 'rw')
            lines = []
            for line in f.readlines():
                if "relative_config" in line:
                    config_text = line.split(']')[0].split('[')[1].split(',')

                    config = [float(x.strip()) for x in config_text]
                    # print config
                    config[0]+=x_offset
                    config[1]+=y_offset
                    config[2]+=z_offset
                    lines.append("  relative_config: ["+','.join(str(x) for x in config)+"]\n")
                    continue
                lines.append(line)

            o = open("../dump/"+file, 'w')
            for line in lines:
                o.write(line);
            o.close()
    			


if __name__ == "__main__":
    if(len(sys.argv)<5):
        print "Needs an object name, the x offset, the y offset and the z offset as arguments."
    else:         
        fix_grasps(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
