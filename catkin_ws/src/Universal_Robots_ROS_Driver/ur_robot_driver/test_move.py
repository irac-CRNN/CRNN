#!/usr/bin/env python

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import socket

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

S = [0., -1.57, 0., -1.57, -3.14, 0.]
L = [0.,-1.57+1.57/6, 0., -1.57, -3.14, 0.]
R = [0.,-1.57-1.57/6, 0., -1.57, -3.14, 0.]

client = None

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

def move_left():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=L, velocities=[0]*6, time_from_start=rospy.Duration(0.5))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_right():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=R, velocities=[0]*6, time_from_start=rospy.Duration(0.5))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise



def main():
    global client
    global S
    global L
    global R

    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print ("Waiting for server...")
        client.wait_for_server()
        print ("Connected to server")
        
        HOST='localhost'
        
        s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s1.bind((HOST, 12350))
        s1.listen(2)
        
        conn, addr = s1.accept()
        
        print("Socket connected")
        
        start_point()

        while conn:
            data = conn.recv(1024).decode("utf-8")
            
            print(data)

            if data == 'point':
                move_left()

            elif data == 'thumb':
                move_right()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        s1.close()
        raise
        
if __name__ == '__main__': main()


