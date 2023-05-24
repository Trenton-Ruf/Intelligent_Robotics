#!/usr/bin/env python

import socket
import time
import math
import numpy as np
import quaternion

# Ros Includes
import rospy
from rosflight_msgs.msg import Command, Attitude
from nav_msgs.msg import Odometry

# Convert euler anges for roll, pitch, and yaw into a quaternion.
def eulerToQuat(roll, pitch, yaw):
    w = math.cos(roll/ 2) * math.cos(pitch/ 2) * math.cos(yaw/ 2) \
        + math.sin(roll/ 2) * math.sin(pitch/ 2) * math.sin(yaw/ 2)
    x = math.sin(roll/ 2) * math.cos(pitch/ 2) * math.cos(yaw/ 2) \
        - math.cos(roll/ 2) * math.sin(pitch/ 2) * math.sin(yaw/ 2)
    y = math.cos(roll/ 2) * math.sin(pitch/ 2) * math.cos(yaw/ 2) \
        + math.sin(roll/ 2) * math.cos(pitch/ 2) * math.sin(yaw/ 2)
    z = math.cos(roll/ 2) * math.cos(pitch/ 2) * math.sin(yaw/ 2) \
        - math.sin(roll/ 2) * math.sin(pitch/ 2) * math.cos(yaw/ 2)
    # print(w + " " + x + " " + y + " " + z)
    return np.quaternion(w,x,y,z)


class stateMachine():
    # Create States
    neutralState    = 0
    upState         = 1
    downState       = 2
    leftState       = 3
    rightState      = 4
    failedState       = 5 # Eyebrows not detected

    stateSocketDict = {"neutral":0,
                       "up":1,
                       "down":2,
                       "left":3,
                       "right":4,
                       "failed":5}


    def __init__(self):
        # Set inital State
        self.initialState = self.neutralState 
        self.currentState = self.initialState
        
        self.prevState = None

        # Timing
        self.rotationInterval = 0.1 # Seconds
        self.startTime = time.time() # Maybe convert to time.time_ns() for precision

        self.orientation = None
        self.setpoint = None

        # Initialize rotation quaternions
        degreesRot = 2
        thetaRot = degreesRot * math.pi / 180
        self.rotateUp = eulerToQuat(0, thetaRot, 0)
        self.rotateDown = eulerToQuat(0, - thetaRot, 0)
        self.rotateLeft = eulerToQuat(- thetaRot, 0, 0)
        self.rotateRight = eulerToQuat(thetaRot, 0, 0)

        # Initialize socket listener
        # Used for recieving eyebrow states from a client
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.s.bind(("0.0.0.0", 8745))
        #self.s.bind(("socket.gethostname()", 8745))
        self.s.bind(("192.168.106.114", 8745))
        self.s.listen(1)
        self.socketConnected = False


    def connectSocket(self):
        self.clientsocket, self.address = self.s.accept()
        print("Connection from " + str(self.address) +" established")
        msg ="Connected to eyebrow_control server" 
        self.clientsocket.send(msg.encode("utf-8"))
        self.socketConnected = True
    

    def setOrientation(self, attitudeData):
        self.orientation = np.quaternion(attitudeData.pose.pose.orientation.w,
                                        attitudeData.pose.pose.orientation.x,
                                        attitudeData.pose.pose.orientation.y,
                                        attitudeData.pose.pose.orientation.z) 

    def stateTransition(self, newState):
        self.currentState = newState


    def rotate(self, rotation):
        # multiply setpoint quaternion by rotation quaternion
        self.setpoint = self.setpoint * rotation
        

    def loop(self):
        # TODO socket timeouts
        if not self.socketConnected:
            self.connectSocket()
            self.currentState = self.failedState
        else:
            msg = self.clientsocket.recv(64)
            self.currentState = self.stateSocketDict[ msg.decode("utf-8") ]

        if (self.startTime + self.rotationInterval <= time.time()):
            # Reset timer
            self.startTime = time.time()

            # Check for Lost State transition
            if self.prevState != self.failedState and self.currentState == self.failedState:
                # TODO Start Altitude Hold
                pass
            elif self.prevState == self.failedState and self.currentState != self.failedState:
                # TODO Stop Altitude Hold
                pass

            if self.currentState == self.neutralState:
                pass # Do nothing
            elif self.currentState == self.upState:
                self.rotate(self.rotateUp)
            elif self.currentState == self.downState:
                self.rotate(self.rotateDown)
            elif self.currentState == self.leftState:
                #self.rotate(self.rotateLeft)
                pass
            elif self.currentState == self.rightState:
                #self.rotate(self.rotateRight)
                pass
            
            self.prevState = self.currentState
            print("Current State: " + str(self.currentState) )

def main():
    """
    try:
        # Init Node
        rospy.init_node('eyebrow_control')
       
        # Create state machine class
        eyebrowMachine = stateMachine()

        # Create attitude listener
        rospy.Subscriber("/fixedwing/truth/NED", Odometry, eyebrowMachine.setOrientation) 

        # Wait for initial orientation
        while(eyebrowMachine.orientation is None):
            pass

        # Set the initial setpoint to be the same as the first orientation
        eyebrowMachine.setpoint = eyebrowMachine.orientation 

        # Create attitude setpoint publisher
        # TODO

        while(True): # TODO change to if no ROS exception
            eyebrowMachine.loop()
      
    except rospy.ServiceExeption, e:
        pass
        # Shutdown here?
    """

    eyebrowMachine = stateMachine()
    while(True): # TODO change to if no ROS exception
        eyebrowMachine.loop()
      
if __name__=="__main__":
    main()
