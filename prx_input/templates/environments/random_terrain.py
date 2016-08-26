import random
import math

random.seed(123);

for x in xrange(-20,20):
      for y in xrange(-20,20):
            if x!=0 or y!=0:
                  roll = random.uniform(-math.pi/24,math.pi/24);
                  pitch = random.uniform(-math.pi/24,math.pi/24);
                  yaw = 0;
                  sinroll = math.sin(roll);
                  sinpitch = math.sin(pitch);
                  sinyaw = math.sin(yaw);
                  cosroll = math.cos(roll);
                  cospitch = math.cos(pitch);
                  cosyaw = math.cos(yaw);
                  q0 = sinroll*cospitch*cosyaw - cosroll*sinpitch*sinyaw;
                  q1 = cosroll*sinpitch*cosyaw + sinroll*cospitch*sinyaw;
                  q2 = cosroll*cospitch*sinyaw - sinroll*sinpitch*cosyaw;
                  q3 = cosroll*cospitch*cosyaw + sinroll*sinpitch*sinyaw;
                  print "    -"
                  print "      name: terrain_"+str(x)+"_"+str(y)
                  print "      collision_geometry:"
                  print "        type: box"
                  print "        dims: [5,5,.5]"
                  print "        material: black"
                  print "      config:"
                  print "        position: ["+str(x*5)+","+str(y*5)+",.5]"
                  print "        orientation: ["+str(q0)+","+str(q1)+","+str(q2)+","+str(q3)+"]"
