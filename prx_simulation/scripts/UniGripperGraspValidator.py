#!/usr/bin/python
import serial
import syslog
import time

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
        self.atmosphericV = [3.02, 3.02, 3.02]
        self.vacuumV = [1.5, 1.5, 1.5]
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

        self.currentV[0] = float(tmpVal[0:4])
        self.currentV[1] = float(tmpVal[5:9])
        self.currentV[2] = float(tmpVal[10:])

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

