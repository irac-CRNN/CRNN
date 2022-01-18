#!/usr/bin/env python
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *
from math import sin
from math import cos
from math import acos
from math import asin
from math import atan2
from math import sqrt
from math import pi
import socket

HOST = '192.168.0.67'
PORT = 12412

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(2)
client_socket, addr = server_socket.accept()

print "Socket connected" 

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=P, velocities=[0]*6, time_from_start=rospy.Duration(0.5))]
    client.send_goal(g)
    try:
        #client.wait_for_result()
        k=1
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

global client
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

try:
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    print "Waiting for server..."
    client.wait_for_server()
    print "Connected to server"    

    while True:
        data = client_socket.recv(1024)
        print(data)

        if data.decode() == 'clear':
            P = [0., -1.57+1.57/3, -1.57/3, -1.57, -3.14, 0.]
        if data.decode() == 'down':
            P = [0., -1.57+1.57/3, -3.14/3, -1.57, -3.14, 0.]
        if data.decode() == 'left':
            P = [0., -1.57+3.14/3, -1.57/3, -1.57, -3.14, 0.]
        if data.decode() == 'right':
            P = [0., -1.57-2.5/3, -1.57/3, -1.57, -3.14, 0.]
        if data.decode() == 'up':
            P = [0., -1.57, 0., -1.57, -3.14, 0.]

        move1()

except KeyboardInterrupt:
    rospy.signal_shutdown("KeyboardInterrupt")
    server_socket.close()
    raise


    
