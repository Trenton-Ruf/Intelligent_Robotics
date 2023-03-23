#!/bin/bash

source ../../devel/setup.bash

rosnode kill /attitude_control

# Move 10 meters above home position with initial velocity
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'fixedwing'
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 20.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 8.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: ''" 

rosrun rosflight_control attitude_control_gazebo.py

#rosservice call /attitude_set "enable: true"
#rosservice call /attitude_set "enable: false"


<<com
# Set Thrust off
rostopic pub -1 /fixedwing/command rosflight_msgs/Command "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
mode: 0
ignore: 7
x: 0.0
y: 0.0
z: 0.0
F: 0.0" 

# go to home postion
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'fixedwing'
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: ''" 

# Thrust Full
rostopic pub -1 /fixedwing/command rosflight_msgs/Command "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
mode: 0
ignore: 7
x: 0.0
y: 0.0
z: 0.0
F: 1.0" 

# Move 10 meters above home position with initial velocity
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'fixedwing'
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 10.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 10.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: ''" 
com
