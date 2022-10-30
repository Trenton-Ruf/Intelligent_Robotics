#!/usr/bin/env python

import rospy
from rosflight_msgs.msg import Command, Barometer
from simple_pid import PID

altitude = None
altitudeSetpoint = 10

# create PID controller
pid = PID(0.0015,0.0004,0.003, setpoint=altitudeSetpoint)
pid.output_limits = (-1, 1) # Aileron 

# Create Message Structure
msg = Command()
# need to ignore Aeileron and Rudder, might ignore throttle if decide to make auto-throttle node
msg.ignore = Command.IGNORE_X | Command.IGNORE_Z 
msg.mode = Command.MODE_PASS_THROUGH 

# Create publisher
publisher = rospy.Publisher("/command",Command,queue_size=1)

def altitudeCallback(baro):
    altitude = baro.altitude
    pid.setpoint = altitudeSetpoint
    #pid.tunings = (1, 1, 0) eventually use for fuzzy or genetic PID tuning

    msg.header.stamp = rospy.Time.now()
    msg.F = 0.7        #Throttle (0,1)
    msg.y = pid(altitude) #* -1
    publisher.publish(msg)
    rospy.loginfo("Altitude:"+str(altitude) + " Elevator:"+str(msg.y) )

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
#rospy.loginfo(msg)
