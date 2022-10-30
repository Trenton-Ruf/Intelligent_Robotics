#!/usr/bin/env python

import rospy
from rosflight_msgs.msg import Command, Barometer
import threading
from simple_pid import PID

altitude = None
altitudeSetpoint = 20

def altitudeCallback(baro):
    altitude = baro.altitude
    rospy.loginfo(altitude)
    pass

def altitudePID():

    # Create publisher
    publisher = rospy.Publisher("/command",Command,queue_size=1)
    rate = rospy.Rate(20) # Default needs to be above 10Hz or Control overide to RC

    msg = Command()
    # need to ignore Aeileron and Rudder, might ignore throttle if decide to make auto-throttle node
    msg.ignore = Command.IGNORE_X | Command.IGNORE_Z 
    msg.mode = Command.MODE_PASS_THROUGH 
    
    # create PID controller
    pid = PID(1,0.1,0.05, setpoint=altitudeSetpoint)
    pid.output_limits = (-1, 1) #Elevator deflection (-1,1)

    while not rospy.is_shutdown():
        if altitude is not None:
            pid.setpoint = altitudeSetpoint
            #pid.tunings = (1, 1, 0) eventually use for fuzzy or genetic PID tuning

            msg.header.stamp = rospy.Time.now()
            msg.F = 0.69        #Throttle (0,1)
            msg.y = pid(altitude)   
            publisher.publish(msg)

            #rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Init Node
        rospy.init_node('altitudePID')

        # Create listener
        rospy.Subscriber("/baro",Barometer, altitudeCallback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

#msg.x = 0.0 #Aeileron deflection (-1,1)
#msg.z = 0.0 #Rudder deflection (-1,1)
