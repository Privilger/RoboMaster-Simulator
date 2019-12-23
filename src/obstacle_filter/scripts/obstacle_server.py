#!/usr/bin/python3
import rospy
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import PoseStamped
from arena_filter import Point, IsPointInArena
import socket
from time import ctime
import math
import _thread


HOST ='192.168.199.109'
PORT = 12318
ADDRESS = (HOST, PORT)

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
serverSocket.bind(ADDRESS)
serverSocket.listen(5)


robot_pose_x = 0 
robot_pose_y = 0 
def selfPoseCallback(data):
	global robot_pose_x
	global robot_pose_y
	robot_pose_x = data.pose.position.x
	robot_pose_y = data.pose.position.y
#	print("in:")
    

def call_rosspin():
    rospy.spin()

def filter_server():
    rospy.init_node('obstacle_filter_server_node', anonymous=True)
    rospy.Subscriber("/amcl_pose", PoseStamped, selfPoseCallback)
    _thread.start_new_thread(call_rosspin,())

    while True:
        rospy.loginfo("x:",robot_pose_x)
        rospy.loginfo("y:",robot_pose_y)
        rospy.loginfo("waiting")
        clientSocket, address = serverSocket.accept()
        while True:
            data = clientSocket.recv(1024)
            if not data:
                break
            replyMsg = str(robot_pose_x)+":"+str(robot_pose_y)
            rospy.loginfo(replyMsg)
            clientSocket.send(replyMsg.encode())

        clientSocket.close()
    serverSocket.close()

if __name__ == '__main__':
    try:
        filter_server()
    except rospy.ROSInterruptException:
        pass
