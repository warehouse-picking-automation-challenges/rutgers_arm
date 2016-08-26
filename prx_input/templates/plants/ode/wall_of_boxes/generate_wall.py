import sys
box_size = [3,3,3]
height = 3
width = 10
f = open("new_wall.yaml","w");
f.write("type: ode_plant\n");
f.write("geometries:\n");
count = 0;
for x in xrange(0,height):
    for y in xrange(0,width):
        count = count + 1;
        f.write("  -\n");
        f.write("    name: box"+str(count)+"\n");
        f.write("    collision_geometry: \n");
        f.write("      type: box\n");
        f.write("      dims: "+str(box_size)+"\n");
        f.write("      material: blue\n");
        f.write("    config: \n");
        config = [50,box_size[1]*((-1*width/2.0)+y)+box_size[1]/2.0,x*box_size[2]+box_size[2]/2.0 ]
        f.write("      position: "+str(config)+"\n");
        f.write("      orientation: [0,0,0,1]\n");
        f.write("    dims: "+str(box_size)+"\n");
        f.write("    mass_type: true\n");
        f.write("    weight: 0.00001\n");
f.write("collide_all: true\n");
