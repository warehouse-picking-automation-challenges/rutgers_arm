import math

rod1 = [[-3.5,0,6.5],[3.5,0,13.5]]   #red
rod2 = [[-6.5,0,9.5],[.5,0,16.5]]    #gray
rod3 = [[-5,3,15],[2,3,8]]           #blue
rod4 = [[-5,-3,15],[2,-3,8]]         #white
rod5 = [[0,5,13],[0,-5,13]]          #black
rod6 = [[-3,5,10],[-3,-5,10]]        #green

def cross(a, b):
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]
    return c;
def midpoint(point1,point2):
    return [(point1[0]+point2[0])*.5,(point1[1]+point2[1])*.5,(point1[2]+point2[2])*.5]

def magnitude(point1,point2):
    return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2+(point1[2]-point2[2])**2)
        
#def angles(point1,point2):
#    diff = [(point2[0]-point1[0]),(point2[1]-point1[1]),(point2[2]-point1[2])]
#    #get yaw
#    angle1 = math.atan2(diff[1],diff[0]);
#    #get pitch
#    angle2 = math.atan2(diff[2],diff[0]);
#    #get roll
#    angle3 = -math.atan2(diff[2],diff[1]);
#    return [angle1,angle2,angle3];
        
        
def get_quat(point1,point2):
    diff = [(point2[0]-point1[0]),(point2[1]-point1[1]),(point2[2]-point1[2])];
    axis = cross([1,0,0],diff);
    magn = magnitude([0,0,0],axis);
    axis[0] = axis[0]/magn;
    axis[1] = axis[1]/magn;
    axis[2] = axis[2]/magn;
#    angle_set = angles(point1,point2)
#    roll = angle_set[2]
#    pitch = angle_set[1]
#    yaw = angle_set[0]
#    roll= roll*.5;
#    pitch=pitch*.5;
#    yaw=yaw*.5;
#    sinroll = math.sin(roll);
#    sinpitch = math.sin(pitch);
#    sinyaw = math.sin(yaw);
#    cosroll = math.cos(roll);
#    cospitch = math.cos(pitch);
#    cosyaw = math.cos(yaw);
#    q[0] = sinroll*cospitch*cosyaw - cosroll*sinpitch*sinyaw;
#    q[1] = cosroll*sinpitch*cosyaw + sinroll*cospitch*sinyaw;
#    q[2] = cosroll*cospitch*sinyaw - sinroll*sinpitch*cosyaw;
#    q[3] = cosroll*cospitch*cosyaw + sinroll*sinpitch*sinyaw;
    angle = math.acos((diff[0])/magnitude([0,0,0],diff));
    q = [0,0,0,0]
    angle *= 0.5;
    sinAngle = math.sin(angle);
    
    q[0] = axis[0] * sinAngle;
    q[1] = axis[1] * sinAngle;
    q[2] = axis[2] * sinAngle;
    q[3] = math.cos(angle);
    return q
        
def compute_things(point1,point2):
    mid = midpoint(point1,point2)
    quat = get_quat(point1,point2)
    magn = magnitude(point1,point2)
    s = 'Magn: '+str(magn)+' Mid: '+str(mid)+' Quat: '+str(quat)
    print s

def compute_things2(point2,point1):
    mid = midpoint(point1,point2)
    quat = get_quat(point1,point2)
    magn = magnitude(point1,point2)
    s = 'Magn: '+str(magn)+' Mid: '+str(mid)+' Quat: '+str(quat)
    print s
        
        
compute_things(rod1[1],rod5[0])
compute_things(rod1[1],rod5[1])
compute_things(rod1[1],rod3[1])
compute_things(rod1[1],rod4[1])
        
compute_things(rod1[0],rod6[0])
compute_things(rod1[0],rod6[1])
compute_things(rod1[0],rod3[1])
compute_things(rod1[0],rod4[1])
        
compute_things(rod2[0],rod6[0])
compute_things(rod2[0],rod6[1])
compute_things(rod2[0],rod3[0])
compute_things(rod2[0],rod4[0])
        
compute_things(rod2[1],rod5[0])
compute_things(rod2[1],rod5[1])
compute_things(rod2[1],rod3[0])
compute_things(rod2[1],rod4[0])
        
compute_things(rod3[0],rod5[0])
compute_things(rod3[0],rod6[0])
        
compute_things(rod3[1],rod5[0])
compute_things(rod3[1],rod6[0])
        
compute_things(rod4[0],rod5[1])
compute_things(rod4[0],rod6[1])
        
compute_things(rod4[1],rod5[1])
compute_things(rod4[1],rod6[1])
print '------------------------------------'



compute_things2(rod1[1],rod5[0])
compute_things2(rod1[1],rod5[1])
compute_things2(rod1[1],rod3[1])
compute_things2(rod1[1],rod4[1])

compute_things2(rod1[0],rod6[0])
compute_things2(rod1[0],rod6[1])
compute_things2(rod1[0],rod3[1])
compute_things2(rod1[0],rod4[1])

compute_things2(rod2[0],rod6[0])
compute_things2(rod2[0],rod6[1])
compute_things2(rod2[0],rod3[0])
compute_things2(rod2[0],rod4[0])

compute_things2(rod2[1],rod5[0])
compute_things2(rod2[1],rod5[1])
compute_things2(rod2[1],rod3[0])
compute_things2(rod2[1],rod4[0])

compute_things2(rod3[0],rod5[0])
compute_things2(rod3[0],rod6[0])

compute_things2(rod3[1],rod5[0])
compute_things2(rod3[1],rod6[0])

compute_things2(rod4[0],rod5[1])
compute_things2(rod4[0],rod6[1])

compute_things2(rod4[1],rod5[1])
compute_things2(rod4[1],rod6[1])
    