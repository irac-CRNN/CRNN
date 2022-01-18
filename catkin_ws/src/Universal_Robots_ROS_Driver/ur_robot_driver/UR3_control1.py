#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import sin
from math import cos
from math import acos
from math import asin
from math import atan2
from math import sqrt
from math import pi
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import socket
import csv
import sys
from datetime import datetime

def get_theta(x,y,z):
    D = (x**2.0 + y**2.0 + z**2.0 - l1**2 - l2**2)/(2.0*l1*l2)
    theta_3 = atan2(-sqrt(1.0 - D**2.0), D)
    theta_2 = atan2(z, sqrt(x**2.0 + y**2.0)) - atan2(l2*sin(theta_3), l1+l2*cos(theta_3))
    theta_1 = atan2(y,x)
    theta_2 -= pi

    result = [theta_1, theta_2, theta_3]
    return result           

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=P, velocities=[0]*6, time_from_start=rospy.Duration(0.5))]
    client.send_goal(g)
    try:
        #client.wait_for_result()
        k = 1
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def start_point(): 
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=S, velocities=[0]*6, time_from_start=rospy.Duration(0.5))]
    client.send_goal(g)
    try:        
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

global client
l1 = 24.365
l2 = 29.86
x = 0.0
y = 0.0
z = 0.0
x_axis = 0.0
y_axis = 0.0
z_axis = l1 + l2 - 1
cnt = 0
theta_6 = 0
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

start_time = time.time()
base_control = []
base_robot = []
shoulder_control = []
shoulder_robot = []
elbow_control = []
elbow_robot = []
wrist3_control = []
wrist3_robot = []
time_check = []

try:
    rospy.init_node("UR3_control", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    pub = rospy.Publisher('talker', String, queue_size=10)
    print "Waiting for server..."
    client.wait_for_server()
    print "Connected to server"
    
except KeyboardInterrupt:
    rospy.signal_shutdown("KeyboardInterrupt")
    raise
    
try:

    while True:

        print(data)
        print(int(datetime.utcnow().strftime('%S%f')) - int(data))

        if data == 'clockwise':
            theta_6 += pi/18 

        elif data == 'counterclockwise':
            theta_6 -= pi/18
   
        elif data == 'down':
            z_axis -= 1
 
        elif data == 'left':
            x_axis -= 1

        elif data == 'right':
            x_axis += 1

        elif data == 'up':
            z_axis += 1

        if data != 'ok':
            if x_axis**2.0 + y_axis**2.0 + z_axis**2.0 >= (l1 + l2 - 0.1)**2.0:
                grad_1 = atan2(z_axis, sqrt(x_axis**2.0 + y_axis**2.0))
                grad_2 = atan2(y_axis, x_axis)
                x_axis = (l1 + l2 - 0.1)*cos(grad_1)*cos(grad_2)
                y_axis = (l1 + l2 - 0.1)*cos(grad_1)*sin(grad_2)
                z_axis = (l1 + l2 - 0.1)*sin(grad_1)   
                theta = get_theta(x_axis, y_axis, z_axis)
                theta_1 = theta[0]
                theta_2 = theta[1] 
                theta_3 = theta[2]
        
            elif x_axis**2.0 + y_axis**2.0 + z_axis**2.0 <= (l2-l1 +0.1)**2.0:
                grad_1 = atan2(z_axis, sqrt(x_axis**2.0 + y_axis**2.0))
                grad_2 = atan2(y_axis, x_axis)
                x_axis = (l2-l1 +0.1)*cos(grad_1)*cos(grad_2)
                y_axis = (l2-l1 +0.1)*cos(grad_1)*sin(grad_2)
                z_axis = (l2-l1 +0.1)*sin(grad_1)    
                theta = get_theta(x_axis, y_axis, z_axis)
                theta_1 = theta[0]
                theta_2 = theta[1] 
                theta_3 = theta[2]
        
            else:
                theta = get_theta(x_axis, y_axis, z_axis)
                theta_1 = theta[0]
                theta_2 = theta[1]
                theta_3 = theta[2]
 
            P = [theta_1, theta_2, theta_3, -pi/2, 0., theta_6]
            move1()
            joint_states = rospy.wait_for_message("joint_states", JointState)

            if time.time() - start_time >= 2.5:
                base_control.append(round(theta_1, 4))
                base_robot.append(round(joint_states.position[2], 4))
                shoulder_control.append(round(theta_2, 4))
                shoulder_robot.append(round(joint_states.position[1], 4))
                elbow_control.append(round(theta_3, 4))
                elbow_robot.append(round(joint_states.position[0], 4))
                wrist3_control.append(round(theta_6, 4))
                wrist3_robot.append(round(joint_states.position[5], 4))
                time_check.append(round(time.time() - start_time, 4))

        else:
            x_axis = 0.0
            y_axis = 0.0
            z_axis = l1 + l2 - 1
            theta_6 = 0
            
            if x_axis**2.0 + y_axis**2.0 + z_axis**2.0 >= (l1 + l2 - 0.1)**2.0:
                grad_1 = atan2(z_axis, sqrt(x_axis**2.0 + y_axis**2.0))
                grad_2 = atan2(y_axis, x_axis)
                x_axis = (l1 + l2 - 0.1)*cos(grad_1)*cos(grad_2)
                y_axis = (l1 + l2 - 0.1)*cos(grad_1)*sin(grad_2)
                z_axis = (l1 + l2 - 0.1)*sin(grad_1)   
                theta = get_theta(x_axis, y_axis, z_axis)
                theta_1 = theta[0]
                theta_2 = theta[1] 
                theta_3 = theta[2]
        
            elif x_axis**2.0 + y_axis**2.0 + z_axis**2.0 <= (l2-l1 +0.1)**2.0:
                grad_1 = atan2(z_axis, sqrt(x_axis**2.0 + y_axis**2.0))
                grad_2 = atan2(y_axis, x_axis)
                x_axis = (l2-l1 +0.1)*cos(grad_1)*cos(grad_2)
                y_axis = (l2-l1 +0.1)*cos(grad_1)*sin(grad_2)
                z_axis = (l2-l1 +0.1)*sin(grad_1)    
                theta = get_theta(x_axis, y_axis, z_axis)
                theta_1 = theta[0]
                theta_2 = theta[1] 
                theta_3 = theta[2]
        
            else:
                theta = get_theta(x_axis, y_axis, z_axis)
                theta_1 = theta[0]
                theta_2 = theta[1]
                theta_3 = theta[2]
            
            P = [theta_1, theta_2, theta_3, -pi/2, 0., theta_6]
            move1()

            joint_states = rospy.wait_for_message("joint_states", JointState)

            if time.time() - start_time >= 2.5:
                base_control.append(round(theta_1, 4))
                base_robot.append(round(joint_states.position[2], 4))
                shoulder_control.append(round(theta_2, 4))
                shoulder_robot.append(round(joint_states.position[1], 4))
                elbow_control.append(round(theta_3, 4))
                elbow_robot.append(round(joint_states.position[0], 4))
                wrist3_control.append(round(theta_6, 4))
                wrist3_robot.append(round(joint_states.position[5], 4))
                time_check.append(round(time.time() - start_time, 4))
            
                
except:

    
    raise
        



