#!/usr/bin/env python

import sys 
import rospy
import actionlib
from random import randint
from actionlib_msgs.msg import *
from prx_planning.msg import apc_queryAction, apc_queryGoal

def send_goal(cclient,cgoal):
    cclient.send_goal(cgoal)
    cclient.wait_for_result()
    # rospy.sleep(5)

    if cclient.get_state() != GoalStatus.SUCCEEDED:
        print "@@@ Failed to plan to target configuration"
        return False
    return True

def execute_recorded(cclient):
    init_move(cclient)
    move_to_bin(cclient,'dummy','dummy')

def move(cclient, goal_state):
    cgoal = apc_queryGoal()
    # cgoal.goal_state = [0, 1.57,0,0,-1.70,0,0,0,0, 1.57,0,0,-1.7,0,0,0,1,1]
    cgoal.goal_state = goal_state
    cgoal.stage = apc_queryGoal.MOVE
    if not send_goal(cclient,cgoal):
        return

def init_move(cclient):
    print "Moving to initial position"
    cgoal = apc_queryGoal()
    cgoal.goal_state = [0, 1.57,0,0,-1.70,0,0,0,0, 1.57,0,0,-1.7,0,0,0,1,1]
    # cgoal.goal_state = [0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,1,1]
    cgoal.stage = apc_queryGoal.MOVE
    if not send_goal(cclient,cgoal):
        return

def move_to_bin(cclient,which_hand,which_bin):
    print "Planning for:"+which_hand+"->"+which_bin
    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.MOVE_AND_DETECT
    cgoal.bin = which_bin
    cgoal.hand = which_hand
    if not send_goal(cclient,cgoal):
        return

def right_move(cclient):

    # print "Moving hand->right to bin -> C"
    # cgoal = apc_queryGoal()
    # cgoal.stage = apc_queryGoal.MOVE_AND_DETECT
    # cgoal.bin = "C"
    # cgoal.hand = "right"
    # if not send_goal(cclient,cgoal):
    #     return

    # print "Trying grasp for :"+"right"+"->"+"crayola"
    # cgoal = apc_queryGoal()
    # cgoal.stage = apc_queryGoal.PERFORM_GRASP
    # cgoal.object = "crayola"
    # cgoal.hand = "right"
    # cgoal.bin = "C"
    # # cgoal.object_state = [ 1.1125, 0, 1.3725, 0.707107, 0, 0, 0.707107]
    # # cgoal.object_state = [ 1.1075, 0, 1.4175, 0, 0, 0.707107, 0.707107]
    # cgoal.object_state = [1.012, -0.27,1.452,0.707,0.000,0.707,-0.000]
    # if not send_goal(cclient,cgoal):
    #     return

    # print "Moving hand->right to bin -> B"
    # cgoal = apc_queryGoal()
    # cgoal.stage = apc_queryGoal.MOVE_AND_DETECT
    # cgoal.bin = "B"
    # cgoal.hand = "right"
    # if not send_goal(cclient,cgoal):
    #     return

    # print "Trying grasp for :"+"right"+"->"+"crayola"
    # cgoal = apc_queryGoal()
    # cgoal.stage = apc_queryGoal.PERFORM_GRASP
    # cgoal.object = "crayola"
    # cgoal.hand = "right"
    # cgoal.bin = "B"
    # # cgoal.object_state = [ 1.1125, 0, 1.3725, 0.707107, 0, 0, 0.707107]
    # # cgoal.object_state = [ 1.1075, 0, 1.4175, 0, 0, 0.707107, 0.707107]
    # cgoal.object_state = [1.032, -0.0,1.432,0.707,0.000,0.707,-0.000]
    # if not send_goal(cclient,cgoal):
    #     return

    print "Moving hand->right to bin -> L"
    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.MOVE_AND_DETECT
    cgoal.bin = "L"
    cgoal.hand = "right"
    if not send_goal(cclient,cgoal):
        return

    print "Trying grasp for :"+"right"+"->"+"crayola"
    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.PERFORM_GRASP
    cgoal.object = "crayola"
    cgoal.hand = "right"
    cgoal.bin = "L"
    # cgoal.object_state = [ 1.1125, 0, 1.3725, 0.707107, 0, 0, 0.707107]
    # cgoal.object_state = [ 1.1075, 0, 1.4175, 0, 0, 0.707107, 0.707107]
    cgoal.object_state = [1.032, -0.27,0.710,0.707,0.000,0.707,-0.000]
    if not send_goal(cclient,cgoal):
        return
    

    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.MOVE_TO_ORDER_BIN
    cgoal.hand = "right"
    if not send_goal(cclient,cgoal):
        return

def left_move(cclient):

    print "Moving hand->left to bin -> B"
    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.MOVE_AND_DETECT
    cgoal.bin = "B"
    cgoal.hand = "left"
    if not send_goal(cclient,cgoal):
        return

    print "Trying grasp for :"+"left"+"->"+"crayola"
    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.PERFORM_GRASP
    cgoal.object = "crayola"
    cgoal.hand = "left"
    cgoal.bin = "B"
    cgoal.object_state = [ 1.1125, 0, 1.3725, 0.707107, 0, 0, 0.707107]
    if not send_goal(cclient,cgoal):
        return

    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.MOVE_TO_ORDER_BIN
    cgoal.hand = "left"
    if not send_goal(cclient,cgoal):
        return

if __name__ == '__main__':

    rospy.init_node('test_apc', anonymous=False)
    
    cclient = actionlib.SimpleActionClient('prx/apc_action', apc_queryAction)
    cclient.wait_for_server()

    goals = []
    goals.append([0, 1.57,0,0,-1.70,0,0,0,0, 0,0,0,-1.2,1.3,0,0,1,1])
    goals.append([0, 1.57,0,0,-1.70,0,0,0,0, 1.57,0,0,-1.7,0,0,0,1,1])
    goals.append([0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,1,1])
    goals.append([0, 0,0,0,-1.2,1.3,0,0, 0,1.57,0,0,-1.7,0,0,0,1,1])
    goals.append([0, 1.5, 1.3, 0, 1.2, 0, 1.5, 0, 0,   1.5, 1.3, 0, 1.2, 0, 1.5, 0, 1, 1])
    previ=10



    while True:
        # execute_recorded(cclient);

        # i=randint(0,4)
        # while i==previ:
        #     i=randint(0,4)
        # print goals[i]
        # move(cclient,goals[i])
        # previ=i;
        #Grasping Testing
        init_move(cclient)
        left_move(cclient)
        # init_move(cclient)
        # right_move(cclient)
        # init_move(cclient)

        # Shelf Position Testing
        # move_to_bin(cclient,'left','A')
        # move_to_bin(cclient,'left','B')
        # move_to_bin(cclient,'left','C')
        # move_to_bin(cclient,'left','D')
        # move_to_bin(cclient,'left','E')
        # move_to_bin(cclient,'left','F')
        # move_to_bin(cclient,'left','G')
        # move_to_bin(cclient,'left','H')
        # move_to_bin(cclient,'left','I')
        # move_to_bin(cclient,'left','J')
        # move_to_bin(cclient,'left','K')
        # move_to_bin(cclient,'left','L')
        # init_move(cclient)

        # move_to_bin(cclient,'right','A')
        # move_to_bin(cclient,'right','B')
        # move_to_bin(cclient,'right','C')
        # move_to_bin(cclient,'right','D')
        # move_to_bin(cclient,'right','E')
        # move_to_bin(cclient,'right','F')
        # move_to_bin(cclient,'right','G')
        # move_to_bin(cclient,'right','H')
        # move_to_bin(cclient,'right','I')
        # move_to_bin(cclient,'right','J')
        # move_to_bin(cclient,'right','K')
        # move_to_bin(cclient,'right','L')
        # init_move(cclient)
