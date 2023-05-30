#!/usr/bin/env python

import socket
import math
import numpy as np
import quaternion

# Ros Includes
import rospy
from rosflight_msgs.msg import Command, Attitude
from nav_msgs.msg import Odometry
from rosflight_control.msg import altitudeSet, attitudeSet

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
    rospy.loginfo("Quat " + str(w) + " " + str(x) + " " + str(y) + " " + str(z))
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

        self.orientation = None
        self.setpoint = None

        # Initialize rotation quaternions
        degreesRot = 2
        thetaRot = degreesRot * math.pi / 180
        self.rotateUp = eulerToQuat(0, thetaRot, 0)
        self.rotateDown = eulerToQuat(0, - thetaRot, 0)
        self.rotateLeft = eulerToQuat(- thetaRot, 0, 0)
        self.rotateRight = eulerToQuat(thetaRot, 0, 0)

        # Loop frequency
        self.hz = 1

        # Initialize socket listener
        # Used for recieving eyebrow states from a client
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.s.bind(("0.0.0.0", 8745))
        #self.s.bind(("socket.gethostname()", 8745))
        self.s.bind(("192.168.106.114", 8745))
        self.s.listen(1)
        self.socketConnected = False
        self.s.settimeout( 2/self.hz ) # 2x the loop period 

        # attitudSet Publisher
        self.attitudeSetPub = rospy.Publisher('attitudeSet', attitudeSet, queue_size=1)
        self.attitudeSetMsg = attitudeSet()
        self.attitudeSetMsg.enable = False
        #self.attitudeSetMsg.quaternion = self.orientation


    def connectSocket(self):
        try:
            self.clientsocket, self.address = self.s.accept()
            rospy.loginfo("Connection from " + str(self.address) +" established")
            msg ="Connected to eyebrow_control server" 
            self.clientsocket.send(msg.encode("utf-8"))
            self.socketConnected = True
            return True
        except socket.error:
            self.socketConnected = False
            self.currentState = self.failedState
            rospy.loginfo("connectSocket failed")
            return False


    def recieveSocket(self):
        try:
            msg = self.clientsocket.recv(16)
            self.currentState = self.stateSocketDict[ msg.decode("utf-8") ]
            rospy.loginfo("Socket mesg: " +  msg)
        except:
            #self.socketConnected = False
            self.currentState = self.failedState
            rospy.loginfo("recieveSocket failed")
    

    def setOrientation(self, attitudeData):
        self.orientation = np.quaternion(attitudeData.pose.pose.orientation.w,
                                        attitudeData.pose.pose.orientation.x,
                                        attitudeData.pose.pose.orientation.y,
                                        attitudeData.pose.pose.orientation.z) 


    def rotate(self, rotation):
        # Quaternion rotation p'= q*p*q^-1
        self.setpoint = rotation * self.setpoint # * np.conjugate(rotation)
        rospy.loginfo("rotated quat:" + str(self.setpoint))
            

    def loop(self):
        # Loop Rate
        rate = rospy.Rate(self.hz)

        # Initialize attitudeSet message
        self.attitudeSetMsg.quaternion = quaternion.as_float_array(self.setpoint) #[0]

        while not rospy.is_shutdown():

            self.prevState = self.currentState

            if not self.socketConnected:
                if self.connectSocket():
                    continue
            else:
                self.recieveSocket()

            # Check for Lost State
            if self.currentState == self.failedState:
                # TODO Start Altitude Hold
                # self.attitudeSetMsg.enable = False # uncomment when altitude hold implemented
                pass

            else:
                # TODO Stop Altitude Hold
                self.attitudeSetMsg.enable = True

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
                
                self.attitudeSetMsg.quaternion = quaternion.as_float_array(self.setpoint) #[0]

            rospy.loginfo("Current State: " + str(self.currentState) )

            # Publish Topics
            self.attitudeSetPub.publish(self.attitudeSetMsg)

            # TODO why rate.sleep crashes. Need new thread?
            #rate.sleep()


def main():
    try:
        # Init Node
        rospy.init_node('eyebrow_control')
       
        # Create state machine class
        eyebrowMachine = stateMachine()

        # Create attitude listener
        rospy.Subscriber("/fixedwing/truth/NED", Odometry, eyebrowMachine.setOrientation) 

        # rospy.loginfo("Waiting for initial orientation")
        while(eyebrowMachine.orientation is None):
            pass

        # Set the initial setpoint to be the same as the first orientation
        eyebrowMachine.setpoint = eyebrowMachine.orientation 

        eyebrowMachine.loop()
      
    except rospy.ROSInterruptException:
        # Shutdown here?
        pass
      
if __name__=="__main__":
    main()
