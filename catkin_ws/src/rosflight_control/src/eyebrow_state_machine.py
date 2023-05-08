#!/usr/bin/env python

#import rospy
import time
import math
import numpy as np
import quaternion

def eulerToQuat(roll, pitch, yaw):
    w = math.cos(roll/ 2) * math.cos(pitch/ 2) * math.cos(yaw/ 2) \
        + math.sin(roll/ 2) * math.sin(pitch/ 2) * math.sin(yaw/ 2)
    x = math.sin(roll/ 2) * math.cos(pitch/ 2) * math.cos(yaw/ 2) \
        - math.cos(roll/ 2) * math.sin(pitch/ 2) * math.sin(yaw/ 2)
    y = math.cos(roll/ 2) * math.sin(pitch/ 2) * math.cos(yaw/ 2) \
        + math.sin(roll/ 2) * math.cos(pitch/ 2) * math.sin(yaw/ 2)
    z = math.cos(roll/ 2) * math.cos(pitch/ 2) * math.sin(yaw/ 2) \
        - math.sin(roll/ 2) * math.sin(pitch/ 2) * math.cos(yaw/ 2)
    print(w)
    print(x)
    print(y)
    print(z)
    return 1 # just for testing
    #TODO
    #return quaternion(w,x,y,z)


class stateMachine(initial_orientation):

    def __init__(self):
        # Create States
        self.neutralState    = 0
        self.upState         = 1
        self.downState       = 2
        self.leftState       = 3
        self.rightState      = 4
        self.lostState       = 5 # Eyebrows not detected
        
        # Set inital State
        self.initialState = self.neutralState 
        self.currentState = self.initialState

        # Timing
        rotationInterval = 0.1 # Seconds
        startTime = time.time() # Maybe convert to time.time_ns() for precision

        orientation = initial_orientation

        # Initialize rotation quaternions
        degreesRot = 2
        thetaRot = degreesRot * math.pi / 180
        rotateUp = eulerToQuat(0, thetaRot, 0)
        rotateDown = eulerToQuat(0, - thetaRot, 0)
        rotateLeft = eulerToQuat(- thetaRot, 0, 0)
        rotateRight = eulerToQuat(thetaRot, 0, 0)

        def stateTransition(self, newState):
            self.currentState = newState


        def rotate(self):
            # multiply quaternion by rotation quaternion
            pass

            

        def loop(self):
            if self.currentState == self.neutralState:
                pass
            elif self.currentState == self.upState:
                pass

def main():
    initial_orientation = None
    # Start rospy stuff

    # wait for initial orientation
    while(initial_orientation is None):
        pass

    eyebrowMachine = stateMachine(initial_orientation)
    while(True): # change to if no exception
        eyebrowMachine.loop()
  
if __name__=="__main__":
    main()






    
