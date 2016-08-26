import os.path
import os
import sys
import shutil


list = [1,2,3,4,5,6,7,8,9,10,11,12,16,19,20,29,30,39,40,49,50,59,60,69,70,79,80,89,90,99,100,200,500,1000]
color = ['red', 'green', 'blue', 'yellow', 'pink', 'dark_green', 'light_blue', 'purple', 'teal', 'orange']
c_size = len(color)

init_file = open('simulation_robots.launch','r')
begin_with = init_file.read()

for l in list:
    new_file = open('simulation_'+str(l) +'robots.launch','w')
    new_file.write(begin_with)
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

        new_file.write('\n  <rosparam command="load" ns="simulator/subsystems/consumer/subsystems/router/subsystems/' + disk_name + str(i) + '">')
        
        new_file.write('\n    template: "plant"')
        new_file.write('\n    geometries:')
        new_file.write('\n      -')
        new_file.write('\n        name: body')
        new_file.write('\n        collision_geometry:')
        new_file.write('\n          type: cylinder')
        new_file.write('\n          radius: 4')
        new_file.write('\n          height: 2.0')
        new_file.write('\n          material: ' + color[i%c_size])
        new_file.write('\n  </rosparam>\n')


    new_file.write('\n  <!--node name="simulation" pkg="prx_simulation" type="prx_simulation"')
    new_file.write('\n        required="true" launch-prefix="" output="screen" /-->')
    new_file.write('\n</launch>')


    new_file.close()

