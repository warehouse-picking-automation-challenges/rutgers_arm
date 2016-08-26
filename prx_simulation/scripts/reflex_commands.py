#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
 
from reflex import reflex_smarts
from prx_simulation.srv import *

# To set the correct ethernet interface used for communication with the 
# ReFlex hand if it is different from eth0 (where ethX = correct ethernet interface) run
# (Although it may be simpler to connect the ReFlex to eth0 and communicat with motoman
# via eth1)
#
# rosparam set driver_node/network_interface ethX 
#

# ReFlex Hand finger and joint limits:
# 
# preshape (swivel): 0 to 1.57 (pi/2)
# fingers proximal: 0 to 2.88~
# fingers distal: 0 to 2.27~ (underactuated).

class ReFlexHand():
    def __init__(self):
        self.grasp_service = rospy.Service("/prx/reflex_grasp",gripper_change_srv,self.change_callback)

        # Starts up hand and publishes the command data
        self.reflex_hand = reflex_smarts.ReFlex_Smarts()
        rospy.sleep(1)

        # Zeroing the tactile data is necessary if you wish to use a mode
        # that employs tactile data (like guarded_move) because the
        # sensors drift over time. This creates a callable service proxy 
        zero_tactile = rospy.ServiceProxy('/zero_tactile', Empty)
        zero_tactile()
        rospy.sleep(2)

        # max open angle for inside bin
        self.maxOpenAngle = 1.55

        # max joint angle
        self.maxJointAngle = 3.14

        # max spread angle
        self.maxPreshapeAngle = 1.57

        # Setup grasp modes
        self.GM_basic = 0     # ?? - assume cylindrical
        self.GM_pinch = 1     # pinch
        self.GM_scissor = 2   # equivallent to cylindrical 
        self.GM_wide = 3      # ?? - assume = spherical - fingers equidistant
        self.GM_pinch2 = 4    # a special kind of pinch with thumb tucked in
        self.GM_custom = 5    # allow custom preshape angle

        self.grasping_modes = [self.GM_basic, self.GM_pinch, self.GM_scissor, self.GM_wide, self.GM_pinch2, self.GM_custom]
        # self.setGraspMode(self.GM_basic)

    def change_callback(self,req):
        print "Callback start"
        if req.gripper_state == 1:
            print "order open"
            self.open("order_open")
        elif req.gripper_state == 2:
            print "self closing"
            self.close()
        elif req.gripper_state == 3:
            self.open("bin_open")
            self.setGraspMode( self.GM_basic )
        elif req.gripper_state == 4:
            self.open("bin_open")
            self.setGraspMode( self.GM_pinch )
            self.reflex_hand.move_finger(2, self.maxJointAngle)
        elif req.gripper_state == 5:
            self.close()
        print "Callback end"
        return gripper_change_srvResponse(True)

    # set grasp mode
    def setGraspMode(self, mode, preshapeAngle=0, finger1Angle=0, finger2Angle=0, finger3Angle=0):
        if mode == self.GM_basic:
            # ?? - assume this is cylindrical grasp mode
            #self.reflex_hand.set_cylindrical(1) # doesn't work
            self.reflex_hand.move_preshape(0.01)
            rospy.sleep(1)
            self.open("bin_open")
        elif mode == self.GM_pinch:
            # pinch mode - 2 fingers opposite each other. thumb not used
            #self.reflex_hand.set_pinch(1) # doesn't work
            self.reflex_hand.move_preshape(self.maxPreshapeAngle)
            rospy.sleep(1)
            self.open("bin_open")
        elif mode == self.GM_scissor:
            # ?? - assume this is cylindrical grasp mode
            #self.reflex_hand.set_cylindrical(1) # doesn't work
            self.reflex_hand.move_preshape(0.01)
            rospy.sleep(1)
            self.open("bin_open")
        elif mode == self.GM_wide:
            # ?? - assume this is spherical grasp mode - fingers at 120 degrees from each other
            #self.reflex_hand.set_spherical(1) # doesn't work
            self.reflex_hand.move_preshape(1.05)
            rospy.sleep(1)
            self.open("bin_open")
        elif mode == self.GM_pinch2:
            #self.reflex_hand.set_pinch(1) doesn't work
            self.reflex_hand.move_preshape(self.maxPreshapeAngle)
            rospy.sleep(1)
            self.setJointAngles(self.maxOpenAngle, self.maxOpenAngle, self.maxJointAngle)
        elif mode == self.GM_custom:
            # custom finger preshape position
            self.reflex_hand.move_preshape(preshapeAngle)
            rospy.sleep(1)
            self.setJointAngles(finger1Angle, finger2Angle, finger3Angle)
            
        self._current_grasping_mode = mode



    # set joint angles
    def setJointAngles(self, finger1, finger2, thumb):   
        self.reflex_hand.move_finger(0, finger1)
        self.reflex_hand.move_finger(1, finger2)
        self.reflex_hand.move_finger(2, thumb)
        # self.reflex_hand.command_smarts(1, "f0 " + str(finger1) + " f1 " + str(finger2) + " f2 " + str(thumb))
        # self.reflex_hand.command_smarts(finger1,finger2 ,thumb)

        while any(self.reflex_hand.working) and not rospy.is_shutdown():
            rospy.sleep(0.01)


    # since with all fingers fully extended the hand will not fit inside the shelves
    # we set the fingers to about 80 degrees / 1.4 radians
    def open(self,open_type="order_open"):
        print "open in reflex_commands"
        if open_type == "bin_open":
            self.setJointAngles(self.maxOpenAngle, self.maxOpenAngle, self.maxOpenAngle)
        if open_type == "order_open":
            #self.reflex_hand.command_smarts(1, "open")
            self.setJointAngles(0, 0, 0)
        rospy.sleep(1);


    def close(self):
        print "close in reflex_commands"
        self.setJointAngles(self.maxJointAngle, self.maxJointAngle, self.maxJointAngle);
        #self.setJointAngles(2.3,2.3,2.3);


    def reset(self):
        if self.fake_flag:
            return


    def calibrate(self):
        return

if __name__ == '__main__':
    rospy.init_node('reflex_commands', anonymous=False)
    hand = ReFlexHand()

    r_fast = rospy.Rate(50)
    r_slow = rospy.Rate(1)
    while not rospy.is_shutdown():
        if hand.reflex_hand.hand_publishing:
            r_slow.sleep()
        else:
            hand.reflex_hand._ReFlex__control_loop()
            r_fast.sleep()

