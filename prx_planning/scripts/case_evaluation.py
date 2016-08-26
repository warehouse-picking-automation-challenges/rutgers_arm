#!/usr/bin/env python

import sys 
import rospy
import actionlib
from random import randint
from actionlib_msgs.msg import *
from prx_planning.msg import apc_queryAction, apc_queryGoal

import time
import subprocess

def execute_saved_plan(cclient, object_name, hand, pose, grasp_id, object_state):
    print "Requesting to execute saved plan: "+ object_name +" - "+ hand +" - "+ str(pose) +" - "+ str(grasp_id) +" ::: " + str(object_state)
    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.EXECUTE_SAVED_TRAJ
    cgoal.object=object_name;
    cgoal.hand = hand;
    cgoal.pose = pose;
    cgoal.grasp_id = grasp_id;
    cgoal.object_state = object_state
    return send_goal(cclient,cgoal)

def send_goal(cclient,cgoal):
    cclient.send_goal(cgoal)
    cclient.wait_for_result()

    if cclient.get_state() != GoalStatus.SUCCEEDED:
        print "@@@ Failed to plan to target configuration"
        return False
    return True

def move_home(cclient):
    cgoal = apc_queryGoal()
    cgoal.goal_state = [0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,0.00000,1.57000,0.00000,0.00000,-1.70000,0.00000,0.00000,0.00000,1.00000,1.00000]
    # cgoal.goal_state = [0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    cgoal.stage = apc_queryGoal.MOVE
    return send_goal(cclient,cgoal)

def move_and_detect(cclient, bin_id, hand):
    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.MOVE_AND_DETECT
    cgoal.bin = bin_id
    cgoal.hand = hand
    return send_goal(cclient,cgoal)

def perform_grasp(cclient, bin_id, hand, object_id, object_state):
    print "Trying grasp for :"+hand+"->"+object_id+" at "+str(object_state)
    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.PERFORM_GRASP
    cgoal.object = object_id
    cgoal.hand = hand
    cgoal.bin = bin_id
    cgoal.object_state = object_state
    return send_goal(cclient,cgoal)

def move_to_order_bin(cclient, hand):
    cgoal = apc_queryGoal()
    cgoal.stage = apc_queryGoal.MOVE_TO_ORDER_BIN
    cgoal.hand = hand
    return send_goal(cclient,cgoal)

class video_capture:
    def __init__(self,video_p, video_s):
        self.video_path = video_p
        self.video_source = video_s
    def start_video(self, object_id, arm, pose_id, time_stamp):
        self.video_start_time = str( int( round( time_stamp ) ) )
        args = ['avconv', '-f', 'video4linux2', '-s', '640x480', '-r', '25', '-qscale', '1', '-t', '5', '-i', self.video_source, '-vf','transpose=0,transpose=0',self.video_path + object_id + "_" + arm + "_" + str( pose_id+1) + "_" + str(grasp_id+1) + "_" + self.video_start_time + '.mp4']
        self.video_p = False
        self.video_p = subprocess.Popen(args)
        self.first_video = True

    def stop_video(self):
        self.video_p.terminate()
        self.video_p = False



left_position = [ 1.1125, 0, 1.3925, 0.707107, 0, 0, 0.707107]
right_position = [1.112,-0.270,1.452,0.707,0.000,0.707,0.000]

if __name__ == '__main__':

    rospy.init_node('case_evaluation', anonymous=False)
    cclient = actionlib.SimpleActionClient('prx/apc_action', apc_queryAction)
    cclient.wait_for_server()
    object_id = "crayola_64_ct"
    # object_id = "elmers_washable_no_run_school_glue"
    # object_id = "munchkin_white_hot_duck_bath_toy"
    object_ids = ["champion_copper_plus_spark_plug", "cheezit_big_original" ,"crayola_64_ct" , "dr_browns_bottle_brush", "elmers_washable_no_run_school_glue"  ,"feline_greenies_dental_treats", "highland_6539_self_stick_notes" ,"kong_duck_dog_toy", "kyjen_squekin_eggs_plush_puppies", "mark_twain_huckleberry_finn", "mommys_helper_outlet_plugs", "munchkin_white_hot_duck_bath_toy", "oreo_mega_stuf", "rolodex_jumbo_pencil_cup" ,"stanley_66_052" ];
    hands = ["left","right"];
    # hand = "left"
    vc = video_capture('/media/pracsys/DATA/case_videos/','/dev/video0')
    move_home(cclient)




    bin_id = "B"
    object_position = [0,0,0,0,0,0,0]

    while True:
        object_input = input('Choose the object you want to grasp:\n1.Spark Plug\n2.Cheezit\n3.Crayola\n4.Bottle Brush\n5.School Glue\n6.Feline Greenies\n7.Self Stick Notes\n8.Kong Duck Toy\n9.Squeeking eggs\n10.Huckleberry Finn\n11.Outlet Plugs\n12.Munchkin Duck\n13.Oreo\n14.Pencil Cup\n15.Stanley\n: ');
        object_id = object_ids[int(object_input)-1]
        hand_input = input('Choose which hand you want to use:\n1.Left\n2.Right\n:');
        hand = hands[int(hand_input)-1]
        object_pose_input = input('Choose object pose: (1,2 for bin B) (3,4 for bin E)\n: ' );
        pose = int(object_pose_input)-1
        grasp_id_input = input('Choose the grasp ID (1,2,3)\n:')
        grasp_id = int(grasp_id_input)-1
        # print pose
        # if pose<2:
        #     pose_index = pose+2
        # else:
        #     pose_index = pose+3
        # print pose_index
        pose_index = pose+1
        f = open("case_poses/"+object_id+".yaml");
        pose_count=1
        for line in f:
            line = line.replace(',','')
            if pose_count == pose_index:
                nums = line.split();
                object_position[0] = float(nums[0])
                object_position[1] = float(nums[1])
                object_position[2] = float(nums[2])
                object_position[3] = float(nums[3])
                object_position[4] = float(nums[4])
                object_position[5] = float(nums[5])
                object_position[6] = float(nums[6])
                break
            else:
                pose_count+=1
        f.close()
        global_start_time = time.time()
        vc.start_video(object_id,hand,pose,global_start_time)
        success = execute_saved_plan(cclient, object_id, hand, pose, grasp_id, object_position)
        vc.stop_video()
        
        evaluation_input = raw_input('Was the grasp successful (y or n)?\n:');
        print evaluation_input
        if(evaluation_input.strip().lower() == "y"):
            failure_reason = "Success!"
        else:
            evaluation_input = raw_input('Was the item grasped? (y or n)?\n:');
            if(evaluation_input.strip().lower() == "n"):
                failure_reason = "Failed to grasp item"
            else:
                evaluation_input = raw_input('Was the item dropped during transfer?(y or n)?\n:');
                if(evaluation_input.strip().lower() == "y"):
                    failure_reason = "Item was dropped"
                else:
                    evaluation_input = raw_input('Type the reason the grasp failed?(y or n)?\n:');
                    failure_reason = evaluation_input;

        filename = "case_output.txt"
        f = open( filename, 'a' )
        f.write( str(int( round( global_start_time ) )) + ", " + object_id+ ", " + hand + ", " + str( pose+1 ) + ", "+str(grasp_id+1)+", " + failure_reason + "\n" )
        f.close()
        move_home(cclient)




        # figure out the bin, name and position of the object, and hand
        # next_pose = input('Paused: Give next pose when ready to resume (1,2,3 for bin B) (4,5,6 for bin E): ' );
        # current_pose = int(next_pose)
        # f = open("case_poses/"+object_id+".yaml");

        # pose_count=1
        # for line in f:
        #     if pose_count == current_pose:
        #         nums = line.split();
        #         object_position[0] = float(nums[0])
        #         object_position[1] = float(nums[1])
        #         object_position[2] = float(nums[2])
        #         object_position[3] = float(nums[3])
        #         object_position[4] = float(nums[4])
        #         object_position[5] = float(nums[5])
        #         object_position[6] = float(nums[6])
        #         break
        #     else:
        #         pose_count+=1
        # f.close()

        # if current_pose<=3:
        #     bin_id = "B"
        # else:
        #     bin_id = "E"

        # global_start_time = time.time()
        # init_movement_time = -1;
        # grasp_evaluation_time = -1;
        # move_to_bin_time = -1;
        # failure_reason = "Success"
        # grasp_success = True

        # if move_and_detect(cclient, bin_id, hand):
        #     init_movement_time = time.time()
        #     vc.start_video(object_id,hand,current_pose,global_start_time)
        #     if perform_grasp(cclient, bin_id, hand, object_id, object_position):
        #         grasp_evaluation_time = time.time()
        #         vc.stop_video()
        #         if move_to_order_bin(cclient, hand):
        #             move_to_bin_time = time.time()
        #         else:
        #             failure_reason = "Failed to place object"

        #     else:
        #         failure_reason = "Grasp not found"
        #         vc.stop_video()
        # else:
        #     failure_reason = "Can't approach bin"



        # if failure_reason!="Success":
        #     grasp_success = False;

        # init_movement = init_movement_time - global_start_time
        # grasp_evaluation = grasp_evaluation_time - init_movement_time
        # move_to_bin_diff = move_to_bin_time - grasp_evaluation_time

        # if init_movement < 0:
        #     init_movement = 0
        # if grasp_evaluation < 0:
        #     grasp_evaluation = 0
        # if move_to_bin_diff < 0:
        #     move_to_bin_diff = 0


        # filename = "case_output.txt"
        # f = open( filename, 'a' )
        # f.write( str(int( round( global_start_time ) )) + ", " + object_id+ ", " + hand + ", " + str( current_pose ) + ", " + str( init_movement ) + ", " + str( grasp_evaluation ) + ", " + str( move_to_bin_diff ) + ", "+str(int(grasp_success))+", "+failure_reason + "\n" )
        # f.close()
        # move_home(cclient)