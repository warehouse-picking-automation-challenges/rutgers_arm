#! /usr/bin/env python

import roslib
import rospy
import math

import actionlib

# import actionlib_tutorials.msg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from motoman_msgs.srv import CmdJointTrajectoryEx
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult
)

from servo_device import Controller
from prx_simulation.srv import *

# for UniGripperGraspValidator and UniGripperParallelController
import serial
import syslog
import time
import logging

from std_msgs.msg import Bool


class UniGripperParallelController:
    def __init__(self):

        # Establish connection
        self.port = '/dev/ttyACM5' 
        self.ard = serial.Serial(self.port,9600,timeout=15)
        time.sleep(5) # wait for Arduino

        # we have several state variables for each motor
        self.alarm = [False, False]
        self.inPosition = [False, False]
        self.busy = [False, False]
        self.setOn = [False, False]

        self.debug = False


    # once set to active, arduino will keep sending data every 100ms
    def sendCommand(self, cmd):
        if (self.debug):
            print cmd + '\n\r'

        if (cmd == 'setup' or cmd == 'close'):
            self.ard.write(b's')
            if (self.debug):
                print 'intializes the motors/goto home\n\r'
            time.sleep(4)

        elif (cmd == 'open'):
            self.ard.write(b'0')
            if (self.debug):
                print 'run step 0 -- open all the way\n\r'
            time.sleep(2)

        elif (cmd == 'small'):
            self.ard.write(b'3')
            if (self.debug):
                print 'run step 3 -- small object \n\r'
            time.sleep(3)

        elif (cmd == 'medium'):
            self.ard.write(b'2')
            if (self.debug):
                print 'run step 2 -- medium object \n\r'
            time.sleep(3)

        elif (cmd == 'large'):
            self.ard.write(b'1')
            if (self.debug):
                print 'run step 1 -- large object \n\r'
            time.sleep(3)

        elif (cmd == 'getstatus'):
            self.ard.write(b'g')
            if (self.debug):
                print 'run get status vars command \n\r'


    def checkGrasp(self):
        self.updateStatusVars()
        time.sleep(0.5)

        # if the fingers are no longer moving (not busy) and at least one of them is in position
        # the we have a stable grasp. If the item is on the small side for the specified step
        # one of the fingers may report pushing air / not reached desired position but that is ok
        if (self.busy[0] == False and self.busy[1] == False and (self.inPosition[0] == True or self.inPosition[1] == True)):
            return True
        else: 
            return False


    def updateStatusVars(self):
        # send command
        self.sendCommand('getstatus')

        # Serial read section
        while (True):
            tmpVal = self.ard.readline()

            if (self.debug):
                print 'val len = ' + str(len(tmpVal)) + ' -- val = ' + tmpVal + '\n\r'

            if (len(tmpVal) == 17):
                break

        # parse data and store in correct vars
        for k in range(len(self.alarm)): 
            if (self.debug):
                print tmpVal[0 + k*8:1 + k*8] + ' -- ' + tmpVal[2 + k*8:3 + k*8] + ' -- ' + \
                    tmpVal[4 + k*8:5 + k*8] + ' -- ' + tmpVal[6 + k*8:7 + k*8] + '\r\n'

            self.alarm[k] = bool(int(tmpVal[0 + k*8:1 + k*8]))
            self.inPosition[k] = bool(int(tmpVal[2 + k*8:3 + k*8]))
            self.setOn[k] = bool(int(tmpVal[4 + k*8:5 + k*8]))
            self.busy[k] = bool(int(tmpVal[6 + k*8:7 + k*8]))

        if (self.debug):
            print 'F1: alarm = ' + str(self.alarm[0]) + ', inPos = ' + str(self.inPosition[0]) + ', setOn = ' + str(self.setOn[0]) + ', busy = ' + str(self.busy[0]) + '\n\r' + \
                'F2: alarm = ' + str(self.alarm[1]) + ', inPos = ' + str(self.inPosition[1]) + ', setOn = ' + str(self.setOn[1]) + ', busy = ' + str(self.busy[1])

        # even if not in debug print out alert 
        if (self.alarm[0]):
            print 'ALARM - Finger 1 Alarm -- Reset Controller\r\n'
        if (self.alarm[1]):
            print 'ALARM - Finger 2 Alarm -- Reset Controller\r\n'

        time.sleep(1)

    def noAlarms(self):
        # return false if any alarms are on
        if (self.alarm[0] or self.alarm[1]):
            return False
        else: 
            return True


class UniGripperGraspValidator:
    def __init__(self):

        # Establish connection
        self.port = '/dev/ttyACM2' 
        self.ard = serial.Serial(self.port,9600,timeout=15)
        time.sleep(5) # wait for Arduino

        # we have 3 sensors -- SMC brand
        # sensor 1 (index 0) -- tip vacuum
        # sensor 2 (index 1) -- flat gripper vacuum
        # sensor 3 (index 2) -- grippy vacuum

        # set treshold value in V. If value is atm + treshold the grasp
        # was successful, otherwise it failed 
        self.tresholdV = [0.1, 0.1, 0.1]
        self.threshold =  [0.0, 0.0, 0.0]
        self.atmosphericV = [3.02, 3.02, 3.02]
        self.vacuumV = [1.5, 1.5, 1.5]
        self.lastV = [0.0, 0.0, 0.0]
        self.currentV = [0.0, 0.0, 0.0]
       
        # reset state in case it is still active from before
        self.setActive(False)

    # once set to active, arduino will keep sending data every 100ms
    def setActive(self, active):
        if (active):
            self.ard.write(b'1')
            # print 'set active\n\r'
        else:
            self.ard.write(b'0')
            # print 'set inactive\n\r'

        time.sleep(5);

    # take a few readings in the beginning to calibrate for 
    # current atmospheric pressure readings. Call this before turning on the vacuum
    def calibrateAtmosphericPressure(self):
        self.querySensors()
        for x in [0,1,2]:
            self.atmosphericV[x] = self.currentV[x]
            # print 'calibrating sensor'


    # call this while vacuum is running but before attempting to grasp an object
    def calibrateVacuumPressure(self):
        self.querySensors()
        for x in [0,1,2]:
            self.vacuumV[x] = self.currentV[x]


    def querySensors(self):
        # Serial read section
        #tmpVal = self.ard.readline()

        while (True):
            tmpVal = self.ard.readline()
            # print tmpVal

            if (len(tmpVal) == 16):
                break

        self.currentV[0] = float(tmpVal[10:14])
        self.currentV[1] = float(tmpVal[0:4])
        self.currentV[2] = float(tmpVal[5:9])


        # print 'queried sensors'


    # if not making a good connection the readings can be random stuff
    # if then we try to convert this random stuff from string to float we can get errors
    # this function checks if what we got was really a number
    def isNumber(self, s):
        try:
            float(s)
            return True
        except ValueError:
            return False


    def verifyVacuumGrasp(self, sensor):
        self.querySensors()

        if (self.currentV[sensor] > self.vacuumV[sensor] + self.tresholdV[sensor]):
            return True
        else:
            return False



class MotomanAction(object):
  # create messages that are used to publish feedback/result
  _feedback = FollowJointTrajectoryFeedback()
  _result   = FollowJointTrajectoryResult()

  def __init__(self, name):
    self._action_name = name
    self._names = ['head_hinge']
    self._state = JointState(name=self._names, position=[0]*len(self._names))
    # print self._state.position
    self._joint_limits = {'head_hinge':{'min':0,'max':1.57,'vel':.5}}
    # print self._joint_limits
    self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

    self.servo = Controller()
    self.servo.setAccel(1, 0) # hinge servo
    self.servo.setSpeed(1, 0) # hinge servo
    self.set_servo_state(self._state.position[0]) # head_hinge
    print "@@@@@@@@@@@@@ Initial head_hinge position = ", self._state.position[0]

    self.parallelG = UniGripperParallelController()

    # enable or disable prints
    self.parallelG.debug = False

    # initialize gripper 
    self.parallelG.sendCommand('setup')
    self.parallelG.updateStatusVars()


  # grippy servo 
  def set_servo_state(self, angle):
    # zero angle in servo values = 4000; 90 degrees angle in servo value = 7600
    # value in between are just an approximate...
    one_servo_radian = (7600 - 4000) / (math.pi / 2.0)
    servo_value = int(4000 + angle * one_servo_radian)
    # print "@@@@@@@@@@@@@ servo_value = ", servo_value, "   angle = ", angle
    self.servo.setTarget(1, servo_value)


  # vacuum sensors
  def handle_unigripper_vacuum(self, req):
    global vacuum_on

    if req.TurnVacuumOn == True:
      self.servo.setTarget(0, 7600)
      print "Turned vacuum on"
      return UnigripperVacuumOnResponse(True)
    else:
      self.servo.setTarget(0, 4000)
      print "Turned vacuum off"
      return UnigripperVacuumOnResponse(False)

  def handle_unigripper_vacuum2(self, req):
    global vacuum2_on

    if req.TurnVacuumOn == True:
      self.servo.setTarget(2, 7600)
      print "Turned vacuum 2 on"
      return UnigripperVacuumOnResponse(True)
    else:
      self.servo.setTarget(2, 4000)
      print "Turned vacuum 2 off"
      return UnigripperVacuumOnResponse(False)

  def handle_unigripper_vacuum3(self, req):
    global vacuum3_on
    
    if req.TurnVacuumOn == True:
      self.servo.setTarget(3, 7600)
      print "Turned vacuum 3 on"
      return UnigripperVacuumOnResponse(True)
    else:
      self.servo.setTarget(3, 4000)
      print "Turned vacuum 3 off"
      return UnigripperVacuumOnResponse(False)

  # paralell gripper 
  def parallel_gripper(self, req):
    if req.ParallelCMD == "setup" or \
      req.ParallelCMD == "getstatus" or \
      req.ParallelCMD == "open" or \
      req.ParallelCMD == "close" or \
      req.ParallelCMD == "small" or \
      req.ParallelCMD == "medium" or \
      req.ParallelCMD == "large":
      self.parallelG.sendCommand(req.ParallelCMD)
      return ParallelGripperResponse(True)

  def parallel_gripper_verify_grasp(self, req):
    if (self.self.parallelG.checkGrasp()):
      req.ParallelState = True;
      return ParallelGripperVerifyGraspResponse(True)
    else:
      req.ParallelState = False;
      return ParallelGripperVerifyGraspResponse(False)
    


  def execute_cb(self, goal):
    rospy.loginfo('Received Trajectory')
    ## helper variables
    r = rospy.Rate(100)
    success = True
        
    ## simulate the execution of the trajectory in goal
    trajectory = goal.trajectory
    current_point = trajectory.points[0]

    for x in xrange(len(trajectory.joint_names)):
      if abs(self._state.position[self._state.name.index(trajectory.joint_names[x])] - current_point.positions[x]) > .001:
        rospy.logerr('Simulator: Start State of Joint Trajectory does not correspond to true state of the robot. Aborting trajectory...')
        print "Current: " + str(self._state)
        print "Traj: " + str(current_point)
        print trajectory.joint_names
        success = False
        break

    for outer_loop in xrange(1,len(trajectory.points)):
      target_point = trajectory.points[outer_loop]
      for x in xrange(len(trajectory.joint_names)):
        minimum = self._joint_limits[trajectory.joint_names[x]]['min']
        maximum = self._joint_limits[trajectory.joint_names[x]]['max']
        if target_point.positions[x] < minimum or target_point.positions[x] > maximum:
          rospy.logerr('%s joint position is outside limits. Aborting...',trajectory.joint_names[x])
          success = False
          break
      diff = (target_point.time_from_start - current_point.time_from_start)
      for x in xrange(len(trajectory.joint_names)):
        vel = self._joint_limits[trajectory.joint_names[x]]['vel']
        if math.fabs(target_point.positions[x] - current_point.positions[x])/(diff.secs+diff.nsecs/1000000000.0) > vel:
          print (diff.secs+diff.nsecs/1000000000.0)
          print (target_point.positions[x] - current_point.positions[x])
          print target_point.positions[x]
          print current_point.positions[x]
          rospy.logerr('Unigripper driver:  %s requested velocity %f (given by the time_from_start value) is too large (max %f). Aborting...',trajectory.joint_names[x],(target_point.positions[x] - current_point.positions[x])/(diff.secs+diff.nsecs/1000000000.0),vel)
          success = False
          break

      if not success:
        break
      num_iters = int((diff.secs+diff.nsecs/1000000000.0)/.01)
      # print num_iters
      for iter_count in xrange(num_iters):
        if self._as.is_preempt_requested():
          self._as.set_preempted()
          rospy.loginfo('Preempted')
          success = False
          break
        for x in xrange(len(trajectory.joint_names)):
          val = (target_point.positions[x] - current_point.positions[x])/num_iters
          index = self._state.name.index(trajectory.joint_names[x])
          self._state.position[index] = self._state.position[index] + val

        self.set_servo_state(self._state.position[0])
        r.sleep()
      current_point = target_point

    if success:
      self._as.set_succeeded(self._result)
      rospy.loginfo("SUCCESS!!!")
    else:
      rospy.loginfo("FAILURE :( :( :(");
      
            
if __name__ == '__main__':

  global vacuum_on, vacuum2_on, vacuum3_on 

  vacuum_on = False
  vacuum2_on = False
  vacuum3_on = False

  logger = logging.getLogger('vacuum')
  hdlr = logging.FileHandler('/home/pracsys/repos/vacuum.log')
  formatter = logging.Formatter()
  logger.addHandler(hdlr)
  logger.setLevel(logging.WARNING)


  rospy.init_node('unigripper_joint_path_command')
  action = MotomanAction("unigripper" + rospy.get_name())
  pub = rospy.Publisher('/joint_states', JointState, queue_size=2)
  s = rospy.Service('unigripper_vacuum', UnigripperVacuumOn, action.handle_unigripper_vacuum)
  s1 = rospy.Service('unigripper_vacuum2', UnigripperVacuumOn, action.handle_unigripper_vacuum2)
  s2 = rospy.Service('unigripper_vacuum3', UnigripperVacuumOn, action.handle_unigripper_vacuum3)

  rate = rospy.Rate(10) # 10hz

  s3 = rospy.Service('parallel_gripper', ParallelGripper, action.parallel_gripper)
  s4 = rospy.Service('parallel_grasp_verify', ParallelGripperVerifyGrasp, action.parallel_gripper_verify_grasp)



  # robotiq_state = JointState(name=['finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3', 'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3', 'finger_middle_joint_1', 'finger_middle_joint_2', 'finger_middle_joint_3', 'palm_finger_1_joint', 'palm_finger_2_joint'], position=[0]*11)
  ##publish the state from the MotomanAction 

  action.servo.setTarget(0, 4000)
  action.servo.setTarget(2, 4000)
  action.servo.setTarget(3, 4000)

 # ------ vacuum sensor code -----------------
  uni = UniGripperGraspValidator()
  uni.setActive(True)
  uni.calibrateAtmosphericPressure()

  pub2 = rospy.Publisher('/unigripper_grasp_success', Bool, queue_size=2)

  counter0 = 0
  counter1 = 0
  counter2 = 0
  current_reading = 0
  last_reading = 0
  graspSuccess = False

  reading_list_0 = []
  reading_list_1 = []
  # this here just runs the functions so you can play with various voltages and
  # look at the results

  # print ''.join(['sensor 1 = ', str(uni.currentV[0]), ' -- sensor 2 = ', str(uni.currentV[1]), ' -- sensor 3 = ', str(uni.currentV[2])]

  action._state.header.stamp = rospy.Time.now()

  while not rospy.is_shutdown():
    uni.querySensors()    
    pub.publish(action._state)
    # robotiq_state.header.stamp = rospy.Time.now()
    # pub.publish(robotiq_state)

    #for x in [0,1]:
    # uni.verifyVacuumGrasp(0)
    # uni.verifyVacuumGrasp(1)
    #     # for debug purposes
    if not vacuum_on and not vacuum2_on and not vacuum3_on:
      graspSuccess = False
    threshold = 0.00

    # print ''.join(['grippy lastV = ', str(uni.lastV[0]), ' -- tip lastV = ', str(uni.lastV[1]), ' -- flat lastV = ', str(uni.lastV[2])])
    # print ''.join(['grippy currentV= ', str(uni.currentV[0]), ' -- tip currentV= ', str(uni.currentV[1]), ' -- flat currentV= ', str(uni.currentV[2])])

    if vacuum_on and not graspSuccess:  
      # logger.warning(''.join(['tip = ', str(uni.currentV[0]), ' -- flat = ', str(uni.currentV[1]), ' -- grippy = ', str(uni.currentV[2])]))
      #logging.warning("WTF?")

      if counter == 0 and uni.currentV[0] < uni.lastV[0]-threshold and uni.lastV[0] <= 2.6:
        uni.threshold[0] = uni.lastV[0]
        counter = 1

      if counter >0:
        if uni.currentV[0] < uni.threshold[0]-threshold:
          counter += 1
        else:
          counter =0
          uni.threshold[0] = 0

      if counter > 5:
        print 'set true', uni.currentV[0]
        logger.warning('true')
        logger.warning('set true: %s', uni.currentV[0])
        graspSuccess = True
        counter = 0

    if vacuum2_on and not graspSuccess:
      print "vacuum2_on", vacuum2_on
      if counter2 == 0 and uni.currentV[1] < uni.lastV[1]-threshold and uni.lastV[1] <= 1.9:
        print "found change"
        uni.threshold[1] = uni.lastV[1]
        counter2 = 1

      if counter2 >0:
        if uni.currentV[1] < uni.threshold[1]-threshold:
          print "adding counter1"
          counter2 += 1
        else:
          counter2 = 0
          uni.threshold[1] = 0

      if counter2 > 5:
        # print 'set false'
        print 'set true', uni.currentV[1]
        logger.warning('set true: %s ', uni.currentV[1])
        graspSuccess = True
        counter2 = 0

    if vacuum3_on and not graspSuccess:
      print "vacuum3_on", vacuum3_on
      if counter3 == 0 and uni.currentV[2] < uni.lastV[2]-threshold and uni.lastV[2] <= 2.35:
        uni.threshold[2] = uni.lastV[2]
        counter3 = 1

      if counter3 >0:
        if uni.currentV[2] < uni.threshold[2]-threshold:
          counter3 += 1
        else:
          counter3 = 0
          uni.threshold[2] = 0

      if counter3 > 5:
        # print 'set false'
        print 'set true', uni.currentV[2]
        logger.warning('set true: %s ', uni.currentV[2])
        graspSuccess = True
        counter3 = 0
      # if uni.currentV[0] < uni.threshold[0]-threshold:
      #   counter0+=1
      # if uni.currentV[1] < uni.threshold[1]-threshold:
      #   counter1+=1
      # print 'current_reading: ', current_reading
      # print 'last_reading: ', last_reading
      # threshold = 2.35
      # if count>0:
      # print 'current_reading: ', uni.currentV[0], uni.currentV[1]
      # if current_reading- last_reading > threshold:


    pub2.publish(graspSuccess)
    
    # print ''.join(['grippy lastV = ', str(uni.lastV[0]), ' -- tip lastV = ', str(uni.lastV[1]), ' -- flat lastV = ', str(uni.lastV[2])])

    uni.lastV[0] = uni.currentV[0]
    uni.lastV[1] = uni.currentV[1]
    uni.lastV[2] = uni.currentV[2] #[0.0, 0.0, 0.0]
    # uni.currentV;


    # print ''.join(['sensor 1 = ', str(uni.currentV[0]), ' -- sensor 2 = ', str(uni.currentV[1])])
    # count = count + 1

    rate.sleep()
