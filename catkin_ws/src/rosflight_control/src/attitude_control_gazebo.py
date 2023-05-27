#!/usr/bin/env python

import rospy
from rosflight_msgs.msg import Command, Attitude
from nav_msgs.msg import Odometry
from simple_pid import PID
import numpy as np
import math
import quaternion # using version 2020.11.2.17.0.49 
                    #numba dependency installed with "sudo apt install python-numba"
import rospkg

from rosflight_control.msg import attitudeSet
import time

attitude        = None
aeleronRate     = None
elevatorRate    = None
rudderRate      = None

enable = True;
attitudeSetpoint = np.quaternion(1,0,0,0) 

# unrotated unit quaternion
qU = np.quaternion(1,0,0,0)  

# Atitude Proportional Controller Gain
Kp = 1

# create PID controllers
elevatorPID = PID(0.01,0,0, setpoint=0)
aeleronPID = PID(0,0,0, setpoint=0)
rudderPID = PID(0,0,0, setpoint=0)

elevatorPID.output_limits = (-1,1) # Maximum elevator Deflections
aeleronPID.output_limits = (-1,1) # Maximum aeleron Deflections
rudderPID.output_limits  = (-1,1) # Maximum rudder Deflections

# Create Message Structure
msg = Command()

# msg.ignore = Command.IGNORE_X | Command.IGNORE_Z | Command.IGNORE_F 
#msg.ignore = Command.IGNORE_F # Only ignore throttle at first
msg.F = 0.7 # Just for testing
msg.mode = Command.MODE_PASS_THROUGH 

# Create publisher
publisher = rospy.Publisher("/fixedwing/command",Command,queue_size=1)

startTime = time.time()

def attitudeControl(attitudeData):

    global attitudeSetpoint
    global enable

    if not enable:
        return;

    # Get measured attitude as quaternion
    attitudeMeasured = np.quaternion(attitudeData.pose.pose.orientation.w,
                                    attitudeData.pose.pose.orientation.x,
                                    attitudeData.pose.pose.orientation.y,
                                    attitudeData.pose.pose.orientation.z
                                    ) 

    # rospy.loginfo("attitudeMeasured: " + str(attitudeMeasured))

    # Get the attitude error 
    attitudeError =  np.multiply( np.conjugate(attitudeMeasured),  attitudeSetpoint )

    # Since 2 rotations can describe every attitude,
    # find the shorter of both rotations 
    if attitudeError.w < 0:
        np.negative(attitudeError)

    # Assume derivative of attitude setpoint is proportional to the attitude error
    attitudeSetpointDerivative = Kp * attitudeError

    # Get angular rate setpoints 
    rateSetpoints = np.multiply( (2 * qU) , attitudeSetpointDerivative)

    # Give the PID controllers the new setpoints
    aeleronPID.setpoint  = rateSetpoints.x
    elevatorPID.setpoint = rateSetpoints.y
    rudderPID.setpoint   = rateSetpoints.z

    # Get the Control Surface Deflections from the PID output
    msg.header.stamp = rospy.Time.now()
    msg.x = aeleronPID(attitudeData.twist.twist.angular.x)
    msg.y = elevatorPID(attitudeData.twist.twist.angular.y) 
    msg.z = rudderPID(attitudeData.twist.twist.angular.z) 
    publisher.publish(msg)

    # Send info to the console for debugging
    """
    rospy.loginfo(
                    "Aeleron setpoint:"+str(round(aeleronPID.setpoint, 4)) + 
                    " Elevator setpoint:"+str(round(elevatorPID.setpoint, 4)) + 
                    " Rudder setpoint:"+str(round(rudderPID.setpoint, 4))
                    )

    rospy.loginfo(
                    "Aeleron:"+str(round(msg.x, 4)) + 
                    " Elevator:"+str(round(msg.y, 4)) + 
                    " Rudder:"+str(round(msg.z, 4))
                    )
    """

    #findUltimateGain(elevatorPID,attitudeError.y)
    testZieglerNichols(elevatorPID,attitudeError.y,0.0601,0.218)


def attitudeSet_listener(altitudeSet_data):
    global enable
    enable = altitudeSet_data.enable
    global attitudeSetpoint
    attitudeSetpoint = altitudeSet_data.quaternion

if __name__ == '__main__':
    try:

        # Init Node
        rospy.init_node('attitude_control')

        # Create attitude listener
        #rospy.Subscriber("/fixedwing/attitude", Attitude, attitudeControl)
        rospy.Subscriber("/fixedwing/truth/NED", Odometry, attitudeControl)

        # Create attitudeSet listener
        rospy.Subscriber("attitudeSet", attitudeSet, attitudeSet_listener)

        resetState()
        rospy.spin()

    #except rospy.ROSInterruptException:
    except rospy.ServiceExeption, e:
        pass
