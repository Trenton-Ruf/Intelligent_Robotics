#!/usr/bin/env python

import rospy
from rosflight_msgs.msg import Command, Attitude
from simple_pid import PID
import numpy as np
import math
import quaternion # using version 2020.11.2.17.0.49 
                    #numba dependency installed with "sudo apt install python-numba"

attitude        = None
aeleronRate     = None
elevatorRate    = None
rudderRate      = None

# Initial setpoint value
#attitudeSetpoint = np.quaternion(1,0,0,0) 
#attitudeSetpoint = np.quaternion(0.991,0,0.131,0) # 15 degrees pitch up
attitudeSetpoint = np.quaternion(0.998,0,0.07,0) # 8 degrees pitch up

# unrotated unit quaternion
qU = np.quaternion(1,0,0,0)  

# Atitude Proportional Controller Gain
Kp = 1

# create PID controllers
elevatorPID = PID(0.08,0,0, setpoint=0)
#aeleronPID = PID(0.1,0,0, setpoint=0)
#rudderPID = PID(0.1,0,0, setpoint=0)

aeleronPID = PID(0,0,0, setpoint=0)
rudderPID = PID(0,0,0, setpoint=0)

elevatorPID.output_limits = (-1,1) # Maximum elevator Deflections
aeleronPID.output_limits = (-1,1) # Maximum aeleron Deflections
rudderPID.output_limits  = (-1,1) # Maximum rudder Deflections

# Create Message Structure
msg = Command()

# need to ignore Aeileron, Rudder, and Throttle command outputs.
# msg.ignore = Command.IGNORE_X | Command.IGNORE_Z | Command.IGNORE_F 
#msg.ignore = Command.IGNORE_F # Only ignore throttle at first
msg.F = 1 # Just for testing
msg.mode = Command.MODE_PASS_THROUGH 

# Create publisher
publisher = rospy.Publisher("/fixedwing/command",Command,queue_size=1)

def attitudeControl(attitudeData):

    # Get measured attitude as quaternion
    attitudeMeasured = np.quaternion(attitudeData.attitude.w,
                                    attitudeData.attitude.x,
                                    attitudeData.attitude.y,
                                    attitudeData.attitude.z
                                    ) 

    rospy.loginfo("attitudeMeasured: " + str(attitudeMeasured))

    # Get the attitude error 
    attitudeError =  np.multiply( np.conjugate(attitudeMeasured),  attitudeSetpoint )

    # Since 2 rotations can describe every attitude,
    # find the shorter of both rotations 
    if attitudeError.w < 0:
        np.negative(attitudeError)

    # Assume derivative of attitude setpoint is proportional to the attitude error
    #attitudeSetpointDerivative = np.multiply(Kp, attitudeError)  
    attitudeSetpointDerivative = Kp * attitudeError

    # Get angular rate setpoints 
    rateSetpoints = np.multiply( (2 * qU) , attitudeSetpointDerivative)

    aeleronPID.setpoint  = rateSetpoints.x
    elevatorPID.setpoint = rateSetpoints.y
    rudderPID.setpoint   = rateSetpoints.z

    msg.header.stamp = rospy.Time.now()
    msg.x = aeleronPID(attitudeData.angular_velocity.x) # potentially wrong
    msg.y = elevatorPID(attitudeData.angular_velocity.y) 
    msg.z = rudderPID(attitudeData.angular_velocity.z) # potentially wrong
    publisher.publish(msg)

    # Send info to the console for debugging
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

if __name__ == '__main__':
    try:
        # Init Node
        rospy.init_node('attitude_control')

        # Create attitude listener
        rospy.Subscriber("/fixedwing/attitude", Attitude, attitudeControl)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
