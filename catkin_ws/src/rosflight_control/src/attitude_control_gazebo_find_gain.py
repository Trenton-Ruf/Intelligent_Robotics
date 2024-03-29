#!/usr/bin/env python

import rospy
from rosflight_msgs.msg import Command, Attitude
from nav_msgs.msg import Odometry
from simple_pid import PID
import numpy as np
import math
import quaternion # using version 2020.11.2.17.0.49 
                    #numba dependency installed with "sudo apt install python-numba"
from scipy.signal import find_peaks
from scipy.ndimage.filters import gaussian_filter1d
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState

from rosflight_control.msg import attitudeSet
import time
import matplotlib.pyplot as plt

attitude        = None
aeleronRate     = None
elevatorRate    = None
rudderRate      = None

enable = True;
# Initial setpoint value
attitudeSetpoint = np.quaternion(1,0,0,0) 
#attitudeSetpoint = np.quaternion(0.991,0,0.131,0) # 15 degrees pitch up
#attitudeSetpoint = np.quaternion(0.998,0,0.07,0) # 8 degrees pitch up

# unrotated unit quaternion
qU = np.quaternion(1,0,0,0)  

# Atitude Proportional Controller Gain
Kp = 1

# create PID controllers
elevatorPID = PID(0.03606, 0.33083,0.000983, setpoint=0) # STABLE
aeleronPID = PID(0.03858, 0.52979, 0.00070, setpoint=0)
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

def getFixedwingHeight():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        resp = get_state('fixedwing',"")
        height = float(resp.pose.position.z)
        rospy.loginfo("fixedwing height: "+str(height))
        return height

    except rospy.ServiceExeption, e:
        print("Service call failed: %s" % e)


def resetState():
    state_msg = ModelState()
    state_msg.model_name = 'fixedwing'
    state_msg.pose.position.x = 0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 20
    state_msg.pose.orientation.x = 0 
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0.131
    state_msg.pose.orientation.w = 0.991

    state_msg.twist.linear.x = 8

    rospy.wait_for_service('/gazebo/set_model_state')

    rospy.loginfo("Resetting State")
    
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceExeption, e:
        print("Service call failed: %s" % e)


def plotPID(x,y,gain):
    plt.rcParams["figure.figsize"] = (20,5) 
    title = 'Kp='+str(gain)
    y_gauss = gaussian_filter1d(y, sigma=2)
    #peaks, properties = find_peaks(y_gauss,width=100,prominence=0.2)
    peaks, properties = find_peaks(y_gauss)
    if len(peaks) != 2:
        peak_timestamps= [x[i] for i in peaks if i > 30000]
        if len(peak_timestamps) != 0:
            peak_values = [y_gauss[i] for i in peaks if i > 30000]
            plt.plot(peak_timestamps, peak_values, 'x',label="Peaks")
            avg_period = (peak_timestamps[-1] - peak_timestamps[0]) / len(peak_timestamps) 
            title += (' Period='+str(avg_period))
            rospy.loginfo("Plot Peaks: "+str(peaks))

    plt.plot(x,y, label = "Error Raw")
    plt.plot(x,y_gauss, label = "Error Gauss")
    plt.title(title)
    plt.xlabel('Time')
    plt.legend()
    plt.savefig(str(gain) + '_PID.pdf')
    #plt.show() # Uncomment for testing
    plt.clf()


def findPeriodDeviation(_x,_y):
    deviation = -1
    y_gauss = gaussian_filter1d(_y, sigma=2)
    #peaks, properties = find_peaks(y_gauss,width=100,prominence=0.22)
    peaks, properties = find_peaks(y_gauss)
    rospy.loginfo("Peak indicies: "+str(peaks))
    if len(peaks) > 2:
        peak_timestamps = [_x[i] for i in peaks if i > 30000]
        if len(peak_timestamps) < 10:
            return deviation
        avg_period = (peak_timestamps[-1] - peak_timestamps[0]) / (len(peak_timestamps) -1 ) 
        
        #peak_debug = [_y[i] for i in peaks]
        #rospy.loginfo("Peak error: "+str(peak_debug))

        deviation = 0
        for index,timestamp in enumerate(peak_timestamps[:-1]):
            period = peak_timestamps[index+1] - timestamp
            deviation += abs(avg_period - period)
        deviation/len(peak_timestamps[:-1])
    return deviation

def findHeightDeviation(_x,_y):
    deviation = -1
    y_gauss = gaussian_filter1d(_y, sigma=2)
    #peaks, properties = find_peaks(y_gauss,width=100,prominence=0.22)
    peaks, properties = find_peaks(y_gauss)
    troughs, properties = find_peaks(-y_gauss)
    #rospy.loginfo("Peak indicies: "+str(peaks))
    if len(peaks) > 2:
        peak_heights = [_y[i] for i in peaks if i > 30000]
        trough_depths = [_y[i] for i in troughs if i > 30000]
        if len(peak_heights) < 10:
            return deviation
        avg_height = sum(peak_heights) / len(peak_heights) 
        avg_depth = sum(trough_depths) / len(trough_depths) 
        deviation=0
        for index,height in enumerate(peak_heights):
            deviation += abs(avg_height - height)
        for index,depth in enumerate(trough_depths):
            deviation += abs(avg_depth - depth)
        return deviation * 100 * (avg_height - avg_depth)
    return deviation


ult_x = []
ult_y = []
y=[]
x=[]
best_gain = -1
old_best_gain = -1
inc_gain = 0.10
curr_gain = 0.10
target_gain = 1.0
precision = 5 # decimal points of precision
precision_count = 1
trial_duration = 60
def findUltimateGain(pid,error):
    global best_gain
    global inc_gain
    global curr_gain
    global target_gain
    global old_best_gain
    global startTime
    global ult_x
    global ult_y
    global precision_count

    # Only log the error and the time until trial finished
    elapsed_time = time.time() - startTime
    x.append(elapsed_time)
    y.append(error)
    if elapsed_time > trial_duration: 
        plotPID(x,y,curr_gain) # Debug only
        startTime = time.time()
        rospy.loginfo("curr_gain: "+str(curr_gain))
        rospy.loginfo("best_gain: "+str(best_gain))
        rospy.loginfo("target_gain: "+str(target_gain))
        rospy.loginfo("inc_gain: "+str(inc_gain))
    else:
        return 0


    # Test curr against best
    deviation = findPeriodDeviation(x,y)
    deviation += findHeightDeviation(x,y)
    rospy.loginfo("deviation: "+str(deviation))
    if deviation >=0 and getFixedwingHeight() > 3:
        ult_deviation = findPeriodDeviation(ult_x,ult_y)
        ult_deviation += findHeightDeviation(ult_x,ult_y)
        rospy.loginfo("ult_deviation: "+str(ult_deviation))
        if ult_deviation < 0 or deviation < ult_deviation:
            best_gain = curr_gain
            ult_x = list(x)
            ult_y = list(y)

    # Reset x, y
    del x[:]
    del y[:]

    # if reach end of test interval
    if round(curr_gain, precision) == round(target_gain, precision):
        if precision == precision_count:
            plotPID(ult_x,ult_y,best_gain)
            rospy.signal_shutdown(0)
        curr_gain = float(best_gain - inc_gain)
        target_gain = float(best_gain + inc_gain - (float(inc_gain) / 10))
        inc_gain = (float(inc_gain) / 10)
        old_best_gain = best_gain
        precision_count += 1

    # Increment step
    curr_gain += inc_gain
    if round(curr_gain, precision) == round(old_best_gain, precision):
        curr_gain += inc_gain

    # Disable PID controller
    #pid.auto_mode = False

    # Set new PID proportional gain
    pid.Kp = curr_gain

    # Reset model state
    resetState()

    # Enable PID controller
    #pid.set_auto_mode(True, last_output=0.0)

    return 0


ziegler_started=False
def testZieglerNichols(pid,error,ult_gain,ult_period):
    global ziegler_started 
    global kp
    global ki
    global kd
    if not ziegler_started:
        kp=ult_gain * 0.6
        ki=ult_gain * 1.2 / ult_period
        kd=ult_gain * 0.075 * ult_period
        pid.Kp=kp
        pid.Ki=ki
        pid.Kd=kd
        ziegler_started = True
        resetState()

    elapsed_time = time.time() - startTime
    x.append(elapsed_time)
    y.append(error)
    if elapsed_time > trial_duration: 
        plotPID(x,y, str(kp) + ' Ki='+str(ki) + ' Kd='+str(kd))
        rospy.signal_shutdown(0)



def attitudeControl(attitudeData):

    global attitudeSetpoint

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
    #testZieglerNichols(elevatorPID,attitudeError.y,0.0601,0.218)

    #findUltimateGain( aeleronPID, attitudeError.x )
    #testZieglerNichols( aeleronPID, attitudeError.x , 0.0643, 0.145642573394 )

    #findUltimateGain( rudderPID, attitudeError.z )
    testZieglerNichols( rudderPID, attitudeError.z ,0.7219 ,0.158333940709)


if __name__ == '__main__':
    try:

        # Init Node
        rospy.init_node('attitude_control')

        # Create attitude listener
        #rospy.Subscriber("/fixedwing/attitude", Attitude, attitudeControl)
        rospy.Subscriber("/fixedwing/truth/NED", Odometry, attitudeControl)

        resetState()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
