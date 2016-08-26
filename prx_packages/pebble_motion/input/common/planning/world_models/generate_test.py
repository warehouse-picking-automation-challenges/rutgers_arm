import os.path
import os
import sys
import shutil


list = [1,2,3,4,5,6,7,8,9,10,11,12,16,19,20,29,30,39,40,49,50,59,60,69,70,79,80,89,90,99,100,200,500,1000]

color = ['red', 'green', 'blue', 'yellow', 'pink', 'dark_green', 'light_blue', 'purple', 'teal', 'orange']
c_size = len(color)

init_file = open('world_model_robots.launch','r')
common_text = init_file.read()



for l in list:
    new_file = open('world_model_'+str(l) +'robots.launch','w')

    new_file.write('<launch>')
    new_file.write('\n <rosparam>')
    new_file.write('\n  system_mapping:')
    new_file.write('\n  -') 
    new_file.write('\n    pair: [simulator/consumer/router, world_model/simulator/router]') 
    
    map = ''
    systems = ''
    curr_disk = ''
    for i in range(0,l):      

        disk_name = ''
        if( l > 1000 and i < 10 ):
            disk_name = 'disk000'
        elif( l > 1000 and i < 100 ):
            disk_name = 'disk00'
        elif( l > 1000 and i < 1000 ):
            disk_name = 'disk0'
        elif( l > 100 and i < 10 ):
            disk_name = 'disk00'
        elif( l > 100 and i < 100 ):
            disk_name = 'disk0'
        elif( l > 10 and i < 10 ):
            disk_name = 'disk0'
        else:
            disk_name = 'disk'


        curr_disk = disk_name+str(i)
        map = map + '\n  -'
        map = map + '\n    pair: [simulator/consumer/router/'+ curr_disk +', world_model/simulator/router/' + curr_disk+']'


        systems = systems + '\n  <rosparam command="load" ns="world_model/simulator/subsystems/router/subsystems/'+ curr_disk+ '" file="$(find pebble_motion)/input/common/pebble.yaml"/>'

    new_file.write(map)
    new_file.write('\n\n')
    new_file.write(common_text)
    new_file.write(systems)


    new_file.write('\n\n  <rosparam ns="world_model/simulator/subsystems/router/subsystems/' + curr_disk+ '">')
    new_file.write('\n    embeddings:')
    new_file.write('\n      single_robot_space:')
    new_file.write('\n        type: "full_mapping"')
    new_file.write('\n  </rosparam>')
    new_file.write('\n</launch>')



new_file.close()

