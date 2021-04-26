#!/usr/bin/python
# coding: utf-8
import math
import socket
import rospy
import threading
import time 
from geometry_msgs.msg import Vector3Stamped 
from std_msgs.msg import Float32

global send_msg
global t2
global flag  
global dis

def callback2(data):
    global dis
    dis = data.data


def client():
    global flag
    flag = True
    rospy.init_node('client', anonymous=True)   
    rospy.Subscriber("trans", Float32, callback2)
    rospy.spin()


def tcpip():
    global send_msg
    global dis
    global dis
    yolo_target_pub = rospy.Publisher('camera_detect', Vector3Stamped, queue_size=1)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 连接服务端
    print ('connect state: ', s.connect_ex(('127.0.0.1', 8000)))
    print("go")
    while True:
        receive_msg = s.recv(39).decode()
        #print(len(receive_msg))
        msg = receive_msg.split(',')
        if msg[0] == '1':
            """ QuaternionStamped.x, y, z, w = xmin, ymin, xmax, ymax,   x = time, y= distance """
            cx = 617.537944116307
            cy = 517.929086073535
            f = 1680.49567593274
            xmin = float(msg[1])
            ymin = float(msg[2])
            xmax = float(msg[3])
            ymax = float(msg[4])
            px = (xmin + xmax)/2
            deltax = px-cx
            py = (ymin + ymax)/2
            deltay = py-cy
            # dis = target_corner_msg.pose.position.y
            disz = dis/math.sqrt((abs(deltax)/f)*(abs(deltax)/f)+(abs(deltay)/f)*(abs(deltay)/f)+1)

            disx = disz*deltax/f
            #disy = math.sqrt(dis*dis-disx*disx-disz*disz)
            disy = disz*deltay/f

            send_msg.vector.x = disz-3.93
            send_msg.vector.y = -disx+0.18
            send_msg.vector.z = -disy+0.47
            # target_corner_msg.pose.position.x = float(msg[5]) #time
            #print(time.time()-target_corner_msg.pose.position.x)
        # else:
        #    # print (" target not found ... ")
        #     target_corner_msg.pose.orientation.x = 0
        #     target_corner_msg.pose.orientation.y = 0
        #     target_corner_msg.pose.orientation.z = 0
        #     target_corner_msg.pose.orientation.w = 0
        #     target_corner_msg.pose.position.x = -1

            yolo_target_pub.publish(target_corner_msg)


if __name__ == "__main__":
    client()
