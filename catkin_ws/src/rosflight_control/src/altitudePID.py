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
# need to ignore Aeileron, Rudder, and Throttle
msg.ignore = Command.IGNORE_X | Command.IGNORE_Z | Command.IGNORE_F
msg.mode = Command.MODE_PASS_THROUGH 

# Create publisher
publisher = rospy.Publisher("/command",Command,queue_size=1)

# Recieves the altitude reading message and outputs elevator deflection commands
# Mapping of reading to command is done with a PID controller
def altitudeCallback(baro):
    altitude = baro.altitude
    pid.setpoint = altitudeSetpoint

    msg.header.stamp = rospy.Time.now()
    msg.y = pid(altitude)
    publisher.publish(msg)
    # Log info to console for Debugging
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
