#!/usr/bin/python3
import rospy
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from arena_filter import Point, IsPointInArena
import socket
from time import ctime
import math

HOST ='192.168.199.181'
PORT = 12318
ADDRESS = (HOST, PORT)

clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientSocket.connect(ADDRESS)


#robot_pose_x = 0 
#robot_pose_y = 0 
#def selfPoseCallback(data):
#	robot_pose_x = data.pose.position.x
#	robot_pose_y = data.pose.position.y
    
def IsFriend(friend_point, p):
	dist = math.sqrt((friend_point.x-p.x)*(friend_point.x-p.x)+(friend_point.y-p.y)*(friend_point.y-p.y))
	return dist < 0.15

pub = rospy.Publisher('obstacle_filtered', Obstacles, queue_size=1000)
#dist_pub = rospy.Publisher('enemy_dist', Int8, queue_size=1000)
#friend_pub = rospy.Publisher('friend_pose', PoseStamped, queue_size=1000)
def filterCallback(data):
#    p = str(robot_pose_x) + ':' + str(robot_pose_y)
    p = '1'
    clientSocket.send(p.encode())
    friend_pose = clientSocket.recv(1024)
    if not friend_pose:
        rospy.loginfo('no data')
    else:
        rospy.loginfo("reply:", friend_pose.decode('utf-8'))
        tmp_list = friend_pose.decode('utf-8').split(":")
        friend_point = Point(float(tmp_list[0]), float(tmp_list[1]))
#        friend_position = PoseStamped()
#        friend_position.pose.position.x = friend_point.x
#        friend_position.pose.position.y = friend_point.y
#        friend_pub.publish(friend_position)
    
    filter_msg = Obstacles()
    filter_msg.header.frame_id = "map"
    for it_c in data.circles:
        p = Point(it_c.center.x, it_c.center.y)
        if IsPointInArena(p, 0.2):
            continue
        elif IsFriend(friend_point, p):
            continue
        else:
            filter_msg.circles.append(it_c)
   # print(filter_msg)
    
    pub.publish(filter_msg)

def filter_client():
    rospy.init_node('obstacle_filter_client_node', anonymous=True)
    rospy.Subscriber("/obstacles", Obstacles, filterCallback)
    #rospy.Subscriber("/amcl_pose", PoseStamped, selfPoseCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        filter_client()
    except rospy.ROSInterruptException:
        print('ext')
        pass
