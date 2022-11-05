#!/usr/bin/env python

import rospy
from rosflight_msgs.msg import Command, Barometer
from simple_pid import PID
import numpy as np
import skfuzzy.control as ctrl
import time

altitude = None
altitudeSetpoint = 10

# create PID controller
pid = PID(0.0015,0.0004,0.003, setpoint=altitudeSetpoint)
pid.output_limits =(-1,1) #(-1, 1) # Aileron 
startTime=time.time()
endTime=0
lastPidError = 0

# Create Message Structure
msg = Command()
# need to ignore Aeileron and Rudder, might ignore throttle if decide to make auto-throttle node
msg.ignore = Command.IGNORE_X | Command.IGNORE_Z 
msg.mode = Command.MODE_PASS_THROUGH 

# Create publisher
publisher = rospy.Publisher("/command",Command,queue_size=1)

##########################
# Fuzzy Setup
##########################
#universe = np.linspace(0,1,7) # Discrete one dimentional array

# Create five fuzzy variables - two inputs, three outputs
error = ctrl.Antecedent(np.linspace(-5,5,7), 'error')
delta = ctrl.Antecedent(np.linspace(-40,40,7), 'delta')

"""
kp = ctrl.Consequent(np.linspace(0 ,0.00075,7), 'kp')
kd = ctrl.Consequent(np.linspace(0 ,0.0055,7), 'kd')
ki = ctrl.Consequent(np.linspace(0 ,0.00015,7), 'ki')
"""

"""
# Functional
#error = ctrl.Antecedent(np.linspace(-5,5,7), 'error')
kp = ctrl.Consequent(np.linspace(0 ,0.00075,7), 'kp')
kd = ctrl.Consequent(np.linspace(0 ,0.0055,7), 'kd')
ki = ctrl.Consequent(np.linspace(0 ,0.00015,7), 'ki')
"""
kp = ctrl.Consequent(np.linspace(-0.000000,0.01,7), 'kp')
kd = ctrl.Consequent(np.linspace(-0.000000,0.02,7), 'kd')
ki = ctrl.Consequent(np.linspace(-0.000000,0.01,7), 'ki')

# Fuzzy Terms
names = ['nb', 'nm', 'ns', 'zo', 'ps', 'pm', 'pb']
error.automf(names=names)
delta.automf(names=names)
kp.automf(names=names)
ki.automf(names=names)
kd.automf(names=names)

# Rules lmao here we go

# kp rules ################################################
rule0 = ctrl.Rule(antecedent=((error['nb'] & delta['nb']) |
                              (error['nm'] & delta['nb']) |
                              (error['nb'] & delta['nm']) |
                              (error['nm'] & delta['nm'])),
                  consequent=kp['pb'], label='rule kp pb')

rule1 = ctrl.Rule(antecedent=((error['ns'] & delta['nb']) |
                              (error['zo'] & delta['nb']) |
                              (error['ns'] & delta['nm']) |
                              (error['zo'] & delta['nm']) |
                              (error['nb'] & delta['ns']) |
                              (error['nm'] & delta['ns']) |
                              (error['ns'] & delta['ns']) |
                              (error['nb'] & delta['zo'])),
                  consequent=kp['pm'], label='rule kp pm')

rule2 = ctrl.Rule(antecedent=((error['ps'] & delta['nb']) |
                              (error['ps'] & delta['nm']) |
                              (error['zo'] & delta['ns']) |
                              (error['nm'] & delta['zo']) |
                              (error['ns'] & delta['zo']) |
                              (error['nb'] & delta['ps']) |
                              (error['nm'] & delta['ps']) |
                              (error['nb'] & delta['pm'])),
                  consequent=kp['ps'], label='rule kp ps')

rule3 = ctrl.Rule(antecedent=((error['pm'] & delta['nb']) |
                              (error['pb'] & delta['nb']) |
                              (error['pm'] & delta['nm']) |
                              (error['ps'] & delta['ns']) |
                              (error['zo'] & delta['zo']) |
                              (error['ns'] & delta['ps']) |
                              (error['nm'] & delta['pm']) |
                              (error['nb'] & delta['pb'])),
                  consequent=kp['zo'], label='rule kp zo')

rule4 = ctrl.Rule(antecedent=((error['pb'] & delta['nm']) |
                              (error['pm'] & delta['ns']) |
                              (error['ps'] & delta['zo']) |
                              (error['zo'] & delta['ps']) |
                              (error['ps'] & delta['ps']) |
                              (error['ns'] & delta['pm']) |
                              (error['nm'] & delta['pb'])),
                  consequent=kp['ns'], label='rule kp ns')

rule5 = ctrl.Rule(antecedent=((error['pb'] & delta['ns']) |
                              (error['pm'] & delta['zo']) |
                              (error['pb'] & delta['zo']) |
                              (error['pm'] & delta['ps']) |
                              (error['pb'] & delta['ps']) |
                              (error['pm'] & delta['pm']) |
                              (error['ps'] & delta['pm']) |
                              (error['zo'] & delta['pm']) |
                              (error['ns'] & delta['pb']) |
                              (error['zo'] & delta['pb']) |
                              (error['ps'] & delta['pb'])),
                  consequent=kp['nm'], label='rule kp nm')

rule6 = ctrl.Rule(antecedent=((error['pb'] & delta['pm']) |
                              (error['pm'] & delta['pb']) |
                              (error['pb'] & delta['pb'])),
                  consequent=kp['nb'], label='rule kp nb')

# ki rules ################################################

rule7 = ctrl.Rule(antecedent=((error['nb'] & delta['nb']) |
                              (error['nm'] & delta['nb']) |
                              (error['ns'] & delta['nb']) |
                              (error['nb'] & delta['nm']) |
                              (error['nm'] & delta['nm']) |
                              (error['nb'] & delta['ns'])),
                  consequent=ki['nb'], label='rule ki nb')

rule8 = ctrl.Rule(antecedent=((error['zo'] & delta['nb']) |
                              (error['ns'] & delta['nm']) |
                              (error['nm'] & delta['ns']) |
                              (error['nb'] & delta['zo'])),
                  consequent=ki['nm'], label='rule ki nm')

rule9 = ctrl.Rule(antecedent=((error['ps'] & delta['nb']) |
                              (error['zo'] & delta['nm']) |
                              (error['ps'] & delta['nm']) |
                              (error['ns'] & delta['ns']) |
                              (error['zo'] & delta['ns']) |
                              (error['nm'] & delta['zo']) |
                              (error['ns'] & delta['zo']) |
                              (error['nm'] & delta['ps']) |
                              (error['nb'] & delta['ps'])),
                  consequent=ki['ns'], label='rule ki ns')

rule10 = ctrl.Rule(antecedent=((error['pm'] & delta['nb']) |
                              (error['pb'] & delta['nb']) |
                              (error['pm'] & delta['nm']) |
                              (error['pb'] & delta['nm']) |
                              (error['ps'] & delta['ns']) |
                              (error['zo'] & delta['zo']) |
                              (error['ns'] & delta['ps']) |
                              (error['nm'] & delta['pm']) |
                              (error['nb'] & delta['pm']) |
                              (error['nm'] & delta['pb']) |
                              (error['nb'] & delta['pb'])),
                  consequent=ki['zo'], label='rule ki zo')

rule11 = ctrl.Rule(antecedent=((error['pm'] & delta['ns']) |
                              (error['pb'] & delta['ns']) |
                              (error['ps'] & delta['zo']) |
                              (error['pm'] & delta['zo']) |
                              (error['zo'] & delta['ps']) |
                              (error['ps'] & delta['ps']) |
                              (error['ns'] & delta['pm']) |
                              (error['ns'] & delta['pb'])),
                  consequent=ki['ps'], label='rule ki ps')

rule12 = ctrl.Rule(antecedent=((error['pb'] & delta['zo']) |
                              (error['pm'] & delta['ps']) |
                              (error['ps'] & delta['pm']) |
                              (error['zo'] & delta['pm']) |
                              (error['zo'] & delta['pb'])),
                  consequent=ki['pm'], label='rule ki pm')

rule13 = ctrl.Rule(antecedent=((error['pb'] & delta['ps']) |
                              (error['pm'] & delta['pm']) |
                              (error['pb'] & delta['pm']) |
                              (error['pb'] & delta['pb']) |
                              (error['pm'] & delta['pb']) |
                              (error['ps'] & delta['pb'])),
                  consequent=ki['pb'], label='rule ki pb')

# kd rules ################################################
rule14 = ctrl.Rule(antecedent=((error['nb'] & delta['ns']) |
                              (error['nm'] & delta['ns']) |
                              (error['nb'] & delta['zo']) |
                              (error['nb'] & delta['ps'])),
                  consequent=kd['nb'], label='rule kd nb')

rule15 = ctrl.Rule(antecedent=((error['nb'] & delta['nm']) |
                              (error['ns'] & delta['ns']) |
                              (error['nm'] & delta['zo']) |
                              (error['ns'] & delta['zo']) |
                              (error['nm'] & delta['ps']) |
                              (error['nb'] & delta['pm'])),
                  consequent=kd['nm'], label='rule kd nm')

rule16 = ctrl.Rule(antecedent=((error['nm'] & delta['nm']) |
                              (error['ns'] & delta['nm']) |
                              (error['zo'] & delta['nm']) |
                              (error['ps'] & delta['nm']) |
                              (error['pm'] & delta['nm']) |
                              (error['zo'] & delta['ns']) |
                              (error['zo'] & delta['zo']) |
                              (error['zo'] & delta['ps']) |
                              (error['ns'] & delta['ps']) |
                              (error['zo'] & delta['pm']) |
                              (error['ns'] & delta['pm']) |
                              (error['nm'] & delta['pm'])),
                              #(error['nb'] & delta['pb']) | Test this change , and remove from 'rule kd ps'
                              #(error['nm'] & delta['pb']) | 
                  consequent=kd['ns'], label='rule kd ns')

rule17 = ctrl.Rule(antecedent=((error['ns'] & delta['nb']) |
                              (error['zo'] & delta['nb']) |
                              (error['ps'] & delta['nb']) |
                              (error['ps'] & delta['ns']) |
                              (error['ps'] & delta['zo']) |
                              (error['ps'] & delta['ps']) |
                              (error['ps'] & delta['pm']) |
                              (error['zo'] & delta['pb']) |
                              (error['ns'] & delta['pb'])),
                  consequent=kd['zo'], label='rule kd zo')

rule18 = ctrl.Rule(antecedent=((error['nb'] & delta['nb']) |
                              (error['nm'] & delta['nb']) |
                              (error['pm'] & delta['ns']) |
                              (error['pm'] & delta['zo']) |
                              (error['pm'] & delta['ps']) |
                              (error['pb'] & delta['ps']) |
                              (error['pm'] & delta['pm']) |
                              (error['pb'] & delta['pm']) |
                              (error['nb'] & delta['pb']) | # Really? I'm thinkin delta is 'ns' instead
                              (error['nm'] & delta['pb']) | # Really? I'm thinkin delta is 'ns' instead
                              (error['ps'] & delta['pb'])),
                  consequent=kd['ps'], label='rule kd ps')

rule19 = ctrl.Rule(antecedent=((error['pb'] & delta['nm']) |
                              (error['pb'] & delta['ns']) |
                              (error['pb'] & delta['zo'])),
                  consequent=kd['pm'], label='rule kd pm')

rule20 = ctrl.Rule(antecedent=((error['pm'] & delta['nb']) |
                              (error['pb'] & delta['nb']) |
                              (error['pm'] & delta['pb']) |
                              (error['pb'] & delta['pb'])),
                  consequent=kd['pb'], label='rule kd pb')

system = ctrl.ControlSystem(rules=[ 
                                    rule0,  rule1,  rule2,  rule3,  rule4,  rule5,  rule6, 
                                    rule7,  rule8,  rule9,  rule10, rule11, rule12, rule13,
                                    rule14, rule15, rule16, rule17, rule18, rule19, rule20
                                    ])
#Maybe set clip_to_bounds=False to not limit output to universe
sim = ctrl.ControlSystemSimulation(system, flush_after_run=1000) # lower flush if memory is scarce

altList=[]
def altimeterFilter(baro):
    altList.insert(0,baro.altitude)
    if len(altList) > 3:
        altList.pop()
        avgAltitude = sum(altList) / float(len(altList))
        altitudePID(avgAltitude)


def altitudePID(altitude):

    global startTime
    global endTime
    global lastPidError
    pid.setpoint = altitudeSetpoint

    # Get error and delta
    pidError= altitudeSetpoint - altitude 
    pidDelta = float(pidError - lastPidError)/float(endTime - startTime)
    lastPidError = pidError

    endTime = startTime
    startTime = time.time() 
    
    # Compute Fuzzy Inference
    sim.input['error']= pidError
    sim.input['delta']= pidDelta
    sim.compute()
    #Set pid tunings from fuzzy logic
    pid.tunings = (sim.output['kp'],
                    sim.output['ki'],
                    sim.output['kd']
                    )

    msg.header.stamp = rospy.Time.now()
    msg.F = 1.0        #Throttle (0,1)
    msg.y = pid(altitude) 
    publisher.publish(msg)
    rospy.loginfo("Altitude:"+str(round(altitude, 4)) + 
                    " Elevator:"+str(round(msg.y, 4)) + 
                    " Kp:"+str(round(sim.output['kp'], 4)) + 
                    " Ki:"+str(round(sim.output['ki'], 4)) + 
                    " Kd:"+str(round(sim.output['kd'], 4)) +
                    " Error:"+str(round(pidError)) + 
                    " Delta:"+str(round(pidDelta))
                    )

if __name__ == '__main__':
    try:
        # Init Node
        rospy.init_node('altitudePID')

        # Create listener
        rospy.Subscriber("/baro",Barometer, altimeterFilter)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

#msg.x = 0.0 #Aeileron deflection (-1,1)
#msg.z = 0.0 #Rudder deflection (-1,1)
#rospy.loginfo(msg)
