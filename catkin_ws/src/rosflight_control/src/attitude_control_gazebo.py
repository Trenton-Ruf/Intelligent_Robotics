#!/usr/bin/env python

from simple_pid import PID
import math
import time
import numpy as np
import quaternion # using version 2020.11.2.17.0.49 
                #numba dependency installed with "sudo apt install python-numba"

import rospy
import rospkg
from rosflight_msgs.msg import Command, Attitude
from nav_msgs.msg import Odometry
from rosflight_control.msg import attitudeSet
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

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
elevatorPID = PID(0.03606, 0.33083,0, setpoint=0.000983)
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

def resetState():
    state_msg = ModelState()
    state_msg.model_name = 'fixedwing'
    state_msg.pose.position.x = 0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 20
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0.131
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0.991

    state_msg.twist.linear.x = 8

    rospy.wait_for_service('/gazebo/set_model_state')

    rospy.loginfo("Resetting State")
    
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceExeption, e:
        print("Service call failed: %s" % e)


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


def attitudeSet_listener(attitudeSet_data):
    global enable
    enable = attitudeSet_data.enable
    global attitudeSetpoint
    attitudeSetpoint = quaternion.as_quat_array(attitudeSet_data.quaternion)
    rospy.loginfo("Enable: "+ str(enable) + "\nattitudeSetpoint: " + str(attitudeSetpoint))

if __name__ == '__main__':
    try:

        # Init Node
        rospy.init_node('attitude_control')

        # Create attitude listener
        #rospy.Subscriber("/fixedwing/attitude", Attitude, attitudeControl)
        rospy.Subscriber("/fixedwing/truth/NED", Odometry, attitudeControl)

        # Create attitudeSet listener
        rospy.Subscriber("/attitudeSet", attitudeSet, attitudeSet_listener)

        resetState()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
