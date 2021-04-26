import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import rospy
import numpy as np
import os
import time
import torch
import argparse
from nanodet.util import cfg, load_config, Logger
from nanodet.model.arch import build_model
from nanodet.util import load_model_weight
from nanodet.data.transform import Pipeline

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped 
from std_msgs.msg import Float32
from nlink_parser.msg import LinktrackNodeframe3
import math

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from nlink_parser.msg import LinktrackNodeframe3
from geometry_msgs.msg import Vector3Stamped
from matplotlib import pyplot as plt  # 用来绘制图形
import threading
import copy
from scipy import signal



def callback2(data):
    global dis
    if(data.nodes):
        dis_temp = data.nodes[0].dis
        dis.append(dis_temp)



def plot_real_time():
    global dis
    print(dis)
    plt.plot(dis,color='blue',linestyle='--',label='camera',linewidth=1)
    plt.pause(0.01)  # 暂停一段时间
    plt.ioff()  # 关闭画图窗口



if __name__ == '__main__':
    global dis
    dis = []
    rospy.init_node('listener2', anonymous=True)
    rospy.Subscriber("/nlink_linktrack_nodeframe3", LinktrackNodeframe3, callback2)
    plt.ion()  # 关闭画图窗口
    plt.show() #绘制完后不让窗口关闭
    while True:
        plot_real_time()
        time.sleep(0.001)
    rospy.spin()
