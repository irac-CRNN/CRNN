#!/usr/bin/env python
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *   
import socket

HOST = '127.0.0.1'
PORT = 9999

server_socket = socket.socket(socket.AF_INET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen()
client_socket, addr = server_socket.accept()

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

global client
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

try:
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print "Waiting for server..."
    client.wait_for_server()
    print "Connected to server"    
except KeyboardInterrupt:
    rospy.signal_shutdown("KeyboardInterrupt")
    raise

while True:
    data = client_socket.recv(1024)
    if data.decode() == 0:
        P = [0., 0., 0., 0., 0., 0.]
    if data.decode() == 1:
        P = [0., 0., 0., 0., 0., 0.]
    if data.decode() == 2:
        P = [0., 0., 0., 0., 0., 0.]
    if data.decode() == 3:
        P = [0., 0., 0., 0., 0., 0.]
    if data.decode() == 4:
        P = [0., 0., 0., 0., 0., 0.]
    if data.decode() == 5:
        P = [0., 0., 0., 0., 0., 0.]  
    move1()
    
