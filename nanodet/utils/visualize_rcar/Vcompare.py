from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import rospy
from camera_new.msg import zzw
from camera_new.msg import zw2
import matplotlib.pyplot as plt
import threading
import math

global pub1
global pub2
global all_msg
global dis

def arucocallback(data):
    #uav cordinates
    global all_msg
    all_msg.pose.position.x = (data.pose.position.x)/100
    all_msg.pose.position.y = (data.pose.position.y)/100
    all_msg.pose.position.z = (data.pose.position.z)/100

def transcallback(data):
    #uav cordinates
    global dis
    dis = data.data

def nanocallback(data):
    global pub1
    global all_msg
    global dis
    all_msg.header.stamp = data.header.stamp
    real_x = data.pose.orientation.x/100
    real_z = (data.pose.orientation.z+35)/100
    real_y = math.sqrt(abs(dis*dis - real_x*real_x - real_z*real_z))

    all_msg.pose.orientation.y = -real_x
    all_msg.pose.orientation.z = -(data.pose.orientation.y)*real_y/abs(data.pose.orientation.y)
    all_msg.pose.orientation.x = real_z
    pub1.publish(all_msg)

def viconcallback(data):
    global pub2
    v_msg = PoseStamped()
    v_msg.header.stamp = data.header.stamp
    v_msg.pose.orientation.x = data.pose.position.x
    v_msg.pose.orientation.y = data.pose.position.y
    v_msg.pose.orientation.z = data.pose.position.z
    pub2.publish(v_msg)

def Vcompare():
    global pub1
    global pub2
    global all_msg
    global v_msg
    all_msg = PoseStamped()
    rospy.init_node('compare', anonymous=True)   
    rospy.Subscriber("aruco_detect/send_data_small", zzw, arucocallback)
    rospy.Subscriber("trans",Float32, transcallback)
    rospy.Subscriber("yolo_target_corner", PoseStamped, nanocallback)
    rospy.Subscriber("/mocap/pose", PoseStamped, viconcallback)
    
    pub1 = rospy.Publisher("all",PoseStamped,queue_size=1)
    pub2 = rospy.Publisher("vvv",PoseStamped,queue_size=1)
    print("listening")
    rospy.spin()


if __name__ == "__main__":
    Vcompare()
