#!/usr/bin/env python
# -*- coding:utf-8 -*-
from scipy.spatial.transform import Rotation as R
import rospy
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from scipy import signal
# from scipy.optimize import *
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


class LNode():
    def __init__(self):
        self.data = None
        self.next = None


class MyQueue():
    # 分配头结点
    def __init__(self):
        self.pHead = None  # pHead指向队列首元素
        self.pEnd = None  # pEnd指向队列尾元素

    # 判断队列是否为空，如果为空返回true，否则返回false
    def empty(self):
        self.pHead = None
        self.pEnd = None

    # 获取队列中元素个数
    def size(self):
        size = 0
        p = self.pHead
        while p != None:
            p = p.next
            size += 1
        return size

    # 入队列：把元素e加到队列尾
    def enQueue(self, e):
        p = LNode()
        p.data = copy.deepcopy(e)
        p.next = None
        if self.pHead == None:
            self.pHead = self.pEnd = p
        else:
            self.pEnd.next = p
            self.pEnd = p

    # 获取队首元素，不删除
    def getHead(self):
        if self.pHead == None:
            print("获取队列首元素失败，队列已为空！")
            return None
        return self.pHead

    # 出队列，删除队首元素
    def getFront(self):
        if self.pHead == None:
            print("获取队列首元素失败，队列已为空！")
            return None
        newhead = self.pHead.next
        self.pHead = newhead

    # 取得队列尾元素
    def getBack(self):
        if self.pEnd == None:
            print("获取队列尾元素失败，队列已经为空")
            return None
        return self.pEnd.data

    def showData(self):
        print("show begin")
        print(self.size())
        print("+++++++++++++++++++++++")
        print(self.pHead.data)
        print(self.pEnd.data)
        print("+++++++++++++++++++++++")



    # 将队列转化为3*15数组
    def toArray(self):
        sz = self.size()
        vec = np.ones((3, sz))
        count = self.pHead
        for i in range(sz):
            vec[0][i] = count.data.vector.x
            vec[1][i] = count.data.vector.y
            vec[2][i] = count.data.vector.z
            count = count.next
        return vec

    # 将队列转化为6*15数组
    def toArray3(self):
        sz = self.size()
        vec = np.zeros((6, sz))
        count = self.pHead
        for i in range(sz):
            vec[0][i] = count.data[0]
            vec[1][i] = count.data[1]
            vec[2][i] = count.data[2]
            vec[3][i] = count.data[3]
            vec[4][i] = count.data[4]
            vec[5][i] = count.data[5]
            count = count.next
        return vec

    def toArray2(self):
        sz = self.size()
        vec = np.zeros((3, sz))
        lt = []
        count = self.pHead
        for i in range(sz):
            vec[0][i] = count.data.vector.x
            vec[1][i] = count.data.vector.y
            vec[2][i] = count.data.vector.z
            lt.append(count.data.header.stamp)
            count = count.next
        return vec,lt


    # 获得队尾倒数第二个元素
    def last2(self):
        sz = self.size()
        p = self.pHead
        for i in range(sz - 2):
            p = p.next
        return p

    # 将3*15数组值重新赋值给队列
    def reSign(self, vec):
        # vec2 = Vector3Stamped()
        # for i in range(15):
        #     self.enQueue(vec2)
        count = self.pHead
        for i in range(15):
            count.data.vector.x = vec[0][i]
            count.data.vector.y = vec[1][i]
            count.data.vector.z = vec[2][i]
            count = count.next

    # 将6*15数组值重新赋值给队列
    def reSign2(self, vec):
        vec2 = np.zeros((6, 1))
        for i in range(15):
            self.enQueue(vec2)
        count = self.pHead
        for i in range(15):
            count.data[0] = vec[0][i]
            count.data[1] = vec[1][i]
            count.data[2] = vec[2][i]
            count.data[3] = vec[3][i]
            count.data[4] = vec[4][i]
            count.data[5] = vec[5][i]
            count = count.next

    # 清除到指定队列值之前所有队列
    def clear(self, p):
        while (self.pHead != p):
            self.pHead = self.pHead.next
        self.pHead = self.pHead.next



class mhe:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        """ 
        文件写入
        ######################################################################## 

        estimate.txt    IMU估计
        observe.txt     视觉观测
        mhe.txt         mhe估计
        vicon.txt       真值
        """
        self.estimate = open('/home/mm/zzw/data_analysis/estimate.txt', mode='w')
        self.observe = open('/home/mm/zzw/data_analysis/observe.txt', mode='w')
        self.mhe = open('/home/mm/zzw/data_analysis/mhe.txt', mode='w')
        self.mhe_vel = open('/home/mm/zzw/data_analysis/mhe_vel.txt', mode='w')
        self.vicon = open('/home/mm/zzw/data_analysis/vicon.txt', mode='w')
        self.vicon_vel = open('/home/mm/zzw/data_analysis/vicon_vel.txt', mode='w')
        self.estimate_vel = open('/home/mm/zzw/data_analysis/estimate_vel.txt', mode='w')
        self.observe_vel = open('/home/mm/zzw/data_analysis/observe_vel.txt', mode='w')
        """ 
        话题回调
        ######################################################################## 

        /mavros/imu/data    IMU
        /camera_detect     视觉观测
        /mocap/pose_01      真值
        """
        self.imu = rospy.Subscriber("/mavros/imu/data", Imu, self.callback)
        self.vision_pose = rospy.Subscriber("/camera_detect", Vector3Stamped, self.callback1)
        self.mocap = rospy.Subscriber("/mocap/pose", PoseStamped, self.callback2)
        self.mocap_vel = rospy.Subscriber("/mocap/vel", TwistStamped, self.callback3)

        ########################## Initial index ############################
        self.single_imu_31 = Vector3Stamped()
        self.observe_pos_31 = Vector3Stamped()
        self.observe_vel_31 = Vector3Stamped()
        self.vicon_pos_31 = Vector3Stamped()
        self.vicon_vel_31 = Vector3Stamped()
        self.index = 0
        self.index2 = 0
        self.imu_last = Imu()
        self.initial_vicon_x = 0.0
        self.initial_vicon_y = 0.0
        self.initial_vicon_z = 0.0

        self.initial_obs_x = 0.0
        self.initial_obs_y = 0.0
        self.initial_obs_z = 0.0
        self.flag = True
        ##### mhe zoom ######
        self.zoom = 15
        self.delta_tt = 0.0
        self.vel_all = MyQueue()
        #####################################################################
        self.zoom_imu_acce = MyQueue()
        self.temp_imu_acce = MyQueue()
        self.observe_pos_315 = MyQueue()
        self.observe_vel_315 = MyQueue()
        self.estimate_by_state_transform_615 = MyQueue()
        self.zoom_mhe_615 = MyQueue()
        #####################################################################
        self.estimate_by_state_transform61 = np.zeros((6, 1))
        self.x = np.ones((6, 1))
        self.y = np.ones((6, 1))

    def save_txt(self):

        self.estimate.write('{} {} {} {}\n'.format(self.observe_pos_315.getBack().header.stamp,
                                                    self.estimate_by_state_transform_615.getBack()[0][0],
                                                    self.estimate_by_state_transform_615.getBack()[1][0],
                                                    self.estimate_by_state_transform_615.getBack()[2][0]))

        self.observe.write('{} {} {} {}\n'.format(self.observe_pos_315.getBack().header.stamp,
                                                    self.observe_pos_315.getBack().vector.x,
                                                    self.observe_pos_315.getBack().vector.y,
                                                    self.observe_pos_315.getBack().vector.z))
        self.mhe.write('{} {} {} {}\n'.format(self.observe_pos_315.getBack().header.stamp,
                                                self.zoom_mhe_615.getBack()[0][0],
                                                self.zoom_mhe_615.getBack()[1][0],
                                                self.zoom_mhe_615.getBack()[2][0]))
        self.vicon.write('{} {} {} {}\n'.format(self.vicon_pos_31.header.stamp,
                                                self.vicon_pos_31.vector.x,
                                                self.vicon_pos_31.vector.y,
                                                self.vicon_pos_31.vector.z)) 

        self.estimate_vel.write('{} {} {} {}\n'.format(self.observe_pos_315.getBack().header.stamp,
                                                    self.estimate_by_state_transform_615.getBack()[3][0],
                                                    self.estimate_by_state_transform_615.getBack()[4][0],
                                                    self.estimate_by_state_transform_615.getBack()[5][0]))

        self.observe_vel.write('{} {} {} {}\n'.format(self.observe_pos_315.getBack().header.stamp,
                                                    self.observe_vel_315.getBack().vector.x,
                                                    self.observe_vel_315.getBack().vector.y,
                                                    self.observe_vel_315.getBack().vector.z))


        self.vicon_vel.write('{} {} {} {}\n'.format(self.vicon_pos_31.header.stamp,
                                                self.vicon_vel_31.vector.x,
                                                self.vicon_vel_31.vector.y,
                                                self.vicon_vel_31.vector.z))
    
        self.mhe_vel.write('{} {} {} {}\n'.format(self.observe_pos_315.getBack().header.stamp,
                                                self.zoom_mhe_615.getBack()[3][0],
                                                self.zoom_mhe_615.getBack()[4][0],
                                                self.zoom_mhe_615.getBack()[5][0]))


    def save_plot_data(self):
        global vel_pub,pos_pub
        global obt,obx,oby,obz,obt_vel,obx_vel,oby_vel,obz_vel
        global vit,viz,vix,viy,vit_vel,vix_vel,viy_vel,viz_vel
        global estt,estx,esty,estz,estt_vel,estx_vel,esty_vel,estz_vel
        global mhet,mhex,mhey,mhez,mhet_vel,mhex_vel,mhey_vel,mhez_vel

        mhe_pos_msg = PoseStamped()
        mhe_pos_msg.header.stamp = self.observe_pos_315.getBack().header.stamp
        mhe_pos_msg.pose.position.x = self.zoom_mhe_615.getBack()[0][0]
        mhe_pos_msg.pose.position.y = self.zoom_mhe_615.getBack()[1][0]
        mhe_pos_msg.pose.position.z = self.zoom_mhe_615.getBack()[2][0]
        pos_pub.publish(mhe_pos_msg)

        mhe_vel_msg = TwistStamped()
        mhe_vel_msg.header.stamp = self.observe_pos_315.getBack().header.stamp
        mhe_vel_msg.twist.linear.x = self.zoom_mhe_615.getBack()[3][0]
        mhe_vel_msg.twist.linear.y = self.zoom_mhe_615.getBack()[4][0]
        mhe_vel_msg.twist.linear.z = self.zoom_mhe_615.getBack()[5][0]
        vel_pub.publish(mhe_vel_msg)


        obt.append(self.observe_pos_315.getBack().header.stamp.to_sec())
        obx.append(self.observe_pos_315.getBack().vector.x)
        oby.append(self.observe_pos_315.getBack().vector.y)
        obz.append(self.observe_pos_315.getBack().vector.z)


        obt_vel.append(self.observe_vel_315.getBack().header.stamp.to_sec())
        obx_vel.append(self.observe_vel_315.getBack().vector.x)
        oby_vel.append(self.observe_vel_315.getBack().vector.y)
        obz_vel.append(self.observe_vel_315.getBack().vector.z)


        estt.append(self.observe_pos_315.getBack().header.stamp.to_sec())
        estx.append(self.estimate_by_state_transform_615.getBack()[0][0])
        esty.append(self.estimate_by_state_transform_615.getBack()[1][0])
        estz.append(self.estimate_by_state_transform_615.getBack()[2][0])


        estt_vel.append(self.observe_vel_315.getBack().header.stamp.to_sec())
        estx_vel.append(self.estimate_by_state_transform_615.getBack()[3][0])
        esty_vel.append(self.estimate_by_state_transform_615.getBack()[4][0])
        estz_vel.append(self.estimate_by_state_transform_615.getBack()[5][0])


        mhet.append(self.observe_pos_315.getBack().header.stamp.to_sec())
        mhex.append(self.zoom_mhe_615.getBack()[0][0])
        mhey.append(self.zoom_mhe_615.getBack()[1][0])
        mhez.append(self.zoom_mhe_615.getBack()[2][0])

        mhet_vel.append(self.observe_vel_315.getBack().header.stamp.to_sec())
        mhex_vel.append(self.zoom_mhe_615.getBack()[3][0])
        mhey_vel.append(self.zoom_mhe_615.getBack()[4][0])
        mhez_vel.append(self.zoom_mhe_615.getBack()[5][0])

        vit.append(self.vicon_pos_31.header.stamp.to_sec())
        vix.append(self.vicon_pos_31.vector.x)
        viy.append(self.vicon_pos_31.vector.y)
        viz.append(self.vicon_pos_31.vector.z)

        vit_vel.append(self.vicon_vel_31.header.stamp.to_sec())
        vix_vel.append(self.vicon_vel_31.vector.x)
        viy_vel.append(self.vicon_vel_31.vector.y)
        viz_vel.append(self.vicon_vel_31.vector.z)
        

    #################### mocap_x_11,mocap_y_11,mocap_z_11 賦值 ######
    def callback2(self, mocap):
        self.vicon_pos_31.header.stamp = mocap.header.stamp
        self.vicon_pos_31.vector.x = mocap.pose.position.x
        self.vicon_pos_31.vector.y = mocap.pose.position.y
        self.vicon_pos_31.vector.z = mocap.pose.position.z

        # if(self.index2 == 15):
        #     self.vicon_pos_31.vector.x -= self.initial_vicon_x/15
        #     self.vicon_pos_31.vector.y -= self.initial_vicon_y/15
        #     self.vicon_pos_31.vector.z -= self.initial_vicon_z/15



    def callback3(self,mocap_vel):
        self.vicon_vel_31.header.stamp = mocap_vel.header.stamp
        self.vicon_vel_31.vector.x = mocap_vel.twist.linear.x
        self.vicon_vel_31.vector.y = mocap_vel.twist.linear.y
        self.vicon_vel_31.vector.z = mocap_vel.twist.linear.z
    ################# observe_x_11,observe_y_11,observe_z_11 賦值 #########


    def callback1(self, pose):
        self.observe_pos_31 = pose
        self.observe_vel_31 = copy.deepcopy(pose)
        # if(self.index2 == 15):
        #     self.observe_pos_31.vector.x -= self.initial_obs_x/15
        #     self.observe_pos_31.vector.y -= self.initial_obs_y/15
        #     self.observe_pos_31.vector.z -= self.initial_obs_z/15

        self.observe_pos_31.vector.x = self.observe_pos_31.vector.x-4.3-0.12735+0.05
        self.observe_pos_31.vector.y = self.observe_pos_31.vector.y+0.047-0.03
        self.observe_pos_31.vector.z = self.observe_pos_31.vector.z+0.17+0.113378-0.01



    def callback(self, imu):
        self.imu_last = imu
        # single_imu_31 賦值
        imu_data = np.array([[0], [0], [0]])
        imu_data[0][0] = imu.linear_acceleration.x
        imu_data[1][0] = imu.linear_acceleration.y
        imu_data[2][0] = imu.linear_acceleration.z
        # 初始位姿相對於地磁場的旋轉矩陣
        origin_r = R.from_quat([-0.0151541761012, -0.0014991266006, 0.549382398096, -0.835432273873])  ##(x, y, z, w)
        origin_rotate_inv = origin_r.inv().as_matrix()
        # 任意時刻相對於地磁場的旋轉矩陣
        moving_r = R.from_quat([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
        moving_rotate = moving_r.as_matrix()
        # 任意時刻相對於初始位姿的旋轉矩陣
        result_rotate = moving_rotate * origin_rotate_inv
        imu_data = np.dot(result_rotate, imu_data)
        self.single_imu_31.vector.z = imu_data[2] - 10
        self.single_imu_31.vector.y = imu_data[1]
        self.single_imu_31.vector.x = imu_data[0]  
        self.single_imu_31.header.stamp = imu.header.stamp

        if self.index2 < 15:##赋初值
            self.flag = False
            self.initial_vicon_x += self.vicon_pos_31.vector.x
            self.initial_vicon_y += self.vicon_pos_31.vector.y
            self.initial_vicon_z += self.vicon_pos_31.vector.z

            self.initial_obs_x += self.observe_pos_31.vector.x
            self.initial_obs_y += self.observe_pos_31.vector.y
            self.initial_obs_z += self.observe_pos_31.vector.z
            self.index2 += 1
            # print(self.initial_obs_x)
            return 



        # 队列初始化
        # index =  0: observe_pos_315 和 observe_vel_315 和 zoom_acc_len 的首列赋值

        if self.index == 0:
            
            self.observe_vel_31.vector.x = 0.0
            self.observe_vel_31.vector.y = 0.0
            self.observe_vel_31.vector.z = 0.0
            self.vel_all.enQueue(self.observe_vel_31)
            self.observe_vel_315.enQueue(self.observe_vel_31)
            self.observe_pos_315.enQueue(self.observe_pos_31)
            self.zoom_imu_acce.enQueue(self.single_imu_31)
 
            self.index = self.index + 1

            return

        # 入队直到观测队列size == 15
        ############################ index = 1 ~ 14  ######################
        elif self.index < self.zoom:
            # 如果观测没更新，pass
            if (self.observe_pos_31.header.stamp == self.observe_pos_315.getBack().header.stamp):
                self.temp_imu_acce.enQueue(self.single_imu_31)
            # 观测更新，观测差分得到速度,入队列observe_pos_315和observe_vel_315
            else:
                temp = Vector3Stamped()
                temp_array = self.temp_imu_acce.toArray()
                temp.vector.x = np.mean(temp_array[0,:])
                temp.vector.y = np.mean(temp_array[1,:])
                temp.vector.z = np.mean(temp_array[2,:])
                temp.header.stamp = self.single_imu_31.header.stamp
                self.zoom_imu_acce.enQueue(temp)
                self.temp_imu_acce.empty()

                self.delta_tt = \
                    abs(
                        self.observe_pos_315.getBack().header.stamp.to_sec() - self.observe_pos_31.header.stamp.to_sec())

                
                self.observe_vel_31.vector.x = \
                    (self.observe_pos_31.vector.x - self.observe_pos_315.getBack().vector.x) / self.delta_tt
                self.observe_vel_31.vector.y = \
                    (self.observe_pos_31.vector.y - self.observe_pos_315.getBack().vector.y) / self.delta_tt
                self.observe_vel_31.vector.z = \
                    (self.observe_pos_31.vector.z - self.observe_pos_315.getBack().vector.z) / self.delta_tt


                # temp_imu_acce 加速度队列入队
                self.temp_imu_acce.enQueue(self.single_imu_31)
 
                # observe_pos_315 和 observe_pos_315
                self.observe_pos_315.enQueue(self.observe_pos_31)

                self.vel_all.enQueue(self.observe_vel_31)
                self.observe_vel_315.enQueue(self.observe_vel_31)

                self.index = self.index + 1
            return

        # 队列已满，队尾入列，队头出列，正式开始mhe估计过程
        else:
            # index = 15  zoom = 15
            # 如果观测没更新，pass
            if (self.observe_pos_31.header.stamp == self.observe_pos_315.getBack().header.stamp):
                # zoom_imu_acce 加速度队列入队
                self.temp_imu_acce.enQueue(self.single_imu_31)
                return
            # 如果观测更新
            else:
           

                self.delta_tt = \
                    abs(
                        self.observe_pos_315.getBack().header.stamp.to_sec() - self.observe_pos_31.header.stamp.to_sec())
               

                self.observe_vel_31.vector.x = (self.observe_pos_31.vector.x - self.observe_pos_315.getBack().vector.x) / self.delta_tt
                self.observe_vel_31.vector.y = (self.observe_pos_31.vector.y - self.observe_pos_315.getBack().vector.y) / self.delta_tt
                self.observe_vel_31.vector.z = (self.observe_pos_31.vector.z - self.observe_pos_315.getBack().vector.z) / self.delta_tt

                temp = Vector3Stamped()
                temp_array = self.temp_imu_acce.toArray()
                

                temp.vector.x = np.mean(temp_array[0,:])
                temp.vector.y = np.mean(temp_array[1,:])
                temp.vector.z = np.mean(temp_array[2,:])

                temp.header.stamp = self.single_imu_31.header.stamp
                self.zoom_imu_acce.enQueue(temp)
                self.zoom_imu_acce.getFront()
                self.temp_imu_acce.empty()
                self.temp_imu_acce.enQueue(self.single_imu_31)

                # zoom窗口滚动

                #self.observe_pos_315.showData()
                self.observe_pos_315.enQueue(self.observe_pos_31)
                self.observe_pos_315.getFront()
            

                # self.observe_vel_315.showData()

                self.vel_all.enQueue(self.observe_vel_31)
                self.observe_vel_315.enQueue(self.observe_vel_31)
                self.observe_vel_315.getFront()

                if self.index == self.zoom:
                    for i in range(self.zoom):
                        # index = 15  zoom = 15
                        # estimate_by_state_transform615 队列初始化为观测的observe_pos_315+observe_vel_315的最后一列
                        ax = 0
                        ay = 0
                        az = 0
                        delt = 0.0
                        self.y = self.update_x(ax, ay, az, delt)
                        for j in range(6):
                            self.estimate_by_state_transform61[j] = self.y[j]
                        self.estimate_by_state_transform_615.enQueue(self.estimate_by_state_transform61)

                # index = 16 zoom = 15
                # 状态转移
                # estimate_by_state_transform615 队列滚动

                else:
                    ax = self.zoom_imu_acce.pEnd.data.vector.x
                    ay = self.zoom_imu_acce.pEnd.data.vector.y
                    az = self.zoom_imu_acce.pEnd.data.vector.z

                    self.y = self.update_x(ax, ay, az, self.delta_tt)

                    for j in range(6):
                        self.estimate_by_state_transform61[j] = self.y[j]
                    self.estimate_by_state_transform_615.getFront()
                    self.estimate_by_state_transform_615.enQueue(self.estimate_by_state_transform61)

                # 最小二乘求解
                res = self.mins()
                for i in range(6):
                    self.x[i] = res[i]
                # 更新mhe估计值
                self.zoom_mhe_615.getFront()

                self.zoom_mhe_615.enQueue(self.x)

                # 写入数据

                self.save_plot_data()
                self.save_txt()


    def mins(self):
        r1 = 1#
        r2 = 1#觀測

        # zoom_mhe_615 队列初始化
        # index = 15 zoom = 15
        if self.index == self.zoom:
            temp_pos_315 = self.observe_pos_315.toArray()
            temp_vel_315 = self.observe_vel_315.toArray()
            temp_pv_615 = np.append(temp_pos_315, temp_vel_315, axis=0)
            self.zoom_mhe_615.reSign2(temp_pv_615)
            self.index = self.index + 1
            return temp_pv_615[:, -1]

        # zoom_mhe_615 队列初始化
        # index = 16 zoom = 15
        else:
            temp_estimate_by_state_transform_61 = self.estimate_by_state_transform_615.getBack()
            temp_observe_pos_31 = np.ones((3, 1))
            temp_observe_pos_31[0] = self.observe_pos_315.toArray()[0][-1]
            temp_observe_pos_31[1] = self.observe_pos_315.toArray()[1][-1]
            temp_observe_pos_31[2] = self.observe_pos_315.toArray()[2][-1]

            # 对比滤波前后速度
            # temp_observe_pos_31[0] = self.observe_vel_315.toArray()[0][-1]
            # temp_observe_pos_31[1] = self.observe_vel_315.toArray()[1][-1]
            # temp_observe_pos_31[2] = self.observe_vel_315.toArray()[2][-1]
            # return temp_pv_61
            
            #filtter
            temp_fillter = copy.deepcopy(self.vel_all.toArray())
            # b, a = signal.butter(3, 0.04, 'low',analog=False)
            # temp_fillter[0,:] = signal.filtfilt(b,a,temp_fillter[0,:])
            # temp_fillter[1,:] = signal.filtfilt(b,a,temp_fillter[1,:])
            # temp_fillter[2,:] = signal.filtfilt(b,a,temp_fillter[2,:])


            # self.observe_vel_315.reSign(temp_fillter)
            temp_observe_vel_31 = np.ones((3, 1))
            temp_observe_vel_31[0] = temp_fillter[0][-1]
            temp_observe_vel_31[1] = temp_fillter[1][-1]
            temp_observe_vel_31[2] = temp_fillter[2][-1]

            temp_pv_61 = np.append(temp_observe_pos_31, temp_observe_vel_31, axis=0)

            return (r1 * temp_estimate_by_state_transform_61 + r2 * temp_pv_61) / (r1 + r2)


    # 状态转移
    def update_x(self, ax, ay, az, delt):
        # 初始化
        # index = 15 zoom = 15
        if self.index == self.zoom:
            self.y[0] = self.observe_pos_315.getBack().vector.x
            self.y[1] = self.observe_pos_315.getBack().vector.y
            self.y[2] = self.observe_pos_315.getBack().vector.z
            self.y[3] = self.observe_vel_315.getBack().vector.x
            self.y[4] = self.observe_vel_315.getBack().vector.y
            self.y[5] = self.observe_vel_315.getBack().vector.z

        else:
            A = np.eye(6)
            B = np.zeros((6, 3))
            for i in range(3):
                A[i][i + 3] = delt
                B[i][i] = delt * delt * 0.5
                B[i + 3][i] = delt
            self.y = np.matrix(A) * self.zoom_mhe_615.getBack() + B * np.matrix([ax, ay, az]).T
        return self.y

    def cleanup(self):
        print("shutting down the node")



def plot_real_time():

    global obt,obx,oby,obz,obt_vel,obx_vel,oby_vel,obz_vel
    global vit,viz,vix,viy,vit_vel,vix_vel,viy_vel,viz_vel
    global estt,estx,esty,estz,estt_vel,estx_vel,esty_vel,estz_vel
    global mhet,mhex,mhey,mhez,mhet_vel,mhex_vel,mhey_vel,mhez_vel
    # plt.ion()# 创建实时动态窗口

    if(len(obx)>7000):
        ### pos
        obx = obx[-4000:]
        oby = oby[-4000:]
        obz = obz[-4000:]
        obt = obt[-4000:]
        estt = estt[-4000:]
        mhet = mhet[-4000:]
        estx = estx[-4000:]
        esty = esty[-4000:]
        estz = estz[-4000:]
        mhex = mhex[-4000:]
        mhey = mhey[-4000:]
        mhez = mhez[-4000:]
        vit = vit[-4000:]
        vix = vix[-4000:]
        viy = viy[-4000:]
        viz = viz[-4000:]
        ### vel
        obx_vel =  obx_vel[-4000:]
        oby_vel =  oby_vel[-4000:]
        obz_vel =  obz_vel[-4000:]
        obt_vel =  obt_vel[-4000:]
        estt_vel =  estt_vel[-4000:]
        mhet_vel =  mhet_vel[-4000:]
        estx_vel =  estx_vel[-4000:]
        esty_vel =  esty_vel[-4000:]
        estz_vel =  estz_vel[-4000:]
        mhex_vel =  mhex_vel[-4000:]
        mhey_vel =  mhey_vel[-4000:]
        mhez_vel =  mhez_vel[-4000:]
        vit_vel =  vit_vel[-4000:]
        vix_vel =  vix_vel[-4000:]
        viy_vel =  viy_vel[-4000:]
        viz_vel =  viz_vel[-4000:]


    print("viz:",len(viz))

    if(len(obt)>50):
        plt.clf()  # 清除之前画的图
        fig = plt.gcf()  # 获取当前图

        if(len(obt)<=3000):
            camerat = range(0,len(obt))
            camerax = np.array(obx,dtype=float)
            cameray = np.array(oby,dtype=float)
            cameraz = np.array(obz,dtype=float)

            estimatet = range(0,len(obt))
            estimatex = np.array(estx,dtype=float)
            estimatey = np.array(esty,dtype=float)
            estimatez = np.array(estz,dtype=float)

            mhet1 = range(0,len(obt))
            mhex1 = np.array(mhex,dtype=float)
            mhey1 = np.array(mhey,dtype=float)
            mhez1 = np.array(mhez,dtype=float)

            camerat_vel = range(0,len(obt_vel))
            camerax_vel = np.array(obx_vel,dtype=float)
            cameray_vel = np.array(oby_vel,dtype=float)
            cameraz_vel = np.array(obz_vel,dtype=float)

            estimatet_vel = range(0,len(obt_vel))
            estimatex_vel = np.array(estx_vel,dtype=float)
            estimatey_vel = np.array(esty_vel,dtype=float)
            estimatez_vel = np.array(estz_vel,dtype=float)

            mhet1_vel = range(0,len(obt_vel))
            mhex1_vel = np.array(mhex_vel,dtype=float)
            mhey1_vel = np.array(mhey_vel,dtype=float)
            mhez1_vel = np.array(mhez_vel,dtype=float)


        else:
            camerat = range(0,3000)
            camerax = np.array(obx[-3000:],dtype=float)
            cameray = np.array(oby[-3000:],dtype=float)
            cameraz = np.array(obz[-3000:],dtype=float)

            estimatet = range(0,3000)
            estimatex = np.array(estx[-3000:],dtype=float)
            estimatey = np.array(esty[-3000:],dtype=float)
            estimatez = np.array(estz[-3000:],dtype=float)

            mhet1 = range(0,3000)
            mhex1 = np.array(mhex[-3000:],dtype=float)
            mhey1 = np.array(mhey[-3000:],dtype=float)
            mhez1 = np.array(mhez[-3000:],dtype=float)

            camerat_vel = range(0,3000)
            camerax_vel = np.array(obx_vel[-3000:],dtype=float)
            cameray_vel = np.array(oby_vel[-3000:],dtype=float)
            cameraz_vel = np.array(obz_vel[-3000:],dtype=float)

            estimatet_vel = range(0,3000)
            estimatex_vel = np.array(estx_vel[-3000:],dtype=float)
            estimatey_vel = np.array(esty_vel[-3000:],dtype=float)
            estimatez_vel = np.array(estz_vel[-3000:],dtype=float)

            mhet1_vel = range(0,3000)
            mhex1_vel = np.array(mhex_vel[-3000:],dtype=float)
            mhey1_vel = np.array(mhey_vel[-3000:],dtype=float)
            mhez1_vel = np.array(mhez_vel[-3000:],dtype=float)




        if(len(vit)<=3000):
            vicont = range(0,len(vit))
            viconx = np.array(vix,dtype=float)
            vicony = np.array(viy,dtype=float)
            viconz = np.array(viz,dtype=float)

            vicont_vel = range(0,len(vit_vel))
            viconx_vel = np.array(vix_vel,dtype=float)
            vicony_vel = np.array(viy_vel,dtype=float)
            viconz_vel = np.array(viz_vel,dtype=float)

        else:

            vicont = range(0,3000)
            viconx = np.array(vix[-3000:],dtype=float)
            vicony = np.array(viy[-3000:],dtype=float)
            viconz = np.array(viz[-3000:],dtype=float)

            vicont_vel = range(0,3000)
            viconx_vel = np.array(vix_vel[-3000:],dtype=float)
            vicony_vel = np.array(viy_vel[-3000:],dtype=float)
            viconz_vel = np.array(viz_vel[-3000:],dtype=float)


        len_vicon = len(vicont)
        viconx = viconx[-len_vicon:]
        vicony = vicony[-len_vicon:]
        viconz = viconz[-len_vicon:]


        ax = fig.add_subplot(321)
        plt.plot(camerat,camerax,color='blue',linestyle='--',label='camera',linewidth=1)
        # plt.plot(estimatet,estimatex,color='red',linestyle='--',label='estimate',linewidth=1)
        plt.plot(mhet1,mhex1,color='green',linestyle='--',label='mhe',linewidth=1)
        plt.plot(vicont,viconx,color='red',linestyle='--',label='vicon',linewidth=1)
        plt.grid(True)
        ax.set_xticks([])
        #plt.ylim((-2,2))
        plt.ylabel('x(m)',fontsize=16)


        ax = fig.add_subplot(323)
        plt.plot(camerat,cameray,color='blue',linestyle='--',label='camera',linewidth=1)
        # plt.plot(estimatet,estimatey,color='red',linestyle='--',label='estimate',linewidth=1)
        plt.plot(mhet1,mhey1,color='green',linestyle='--',label='mhe',linewidth=1)
        plt.plot(vicont,vicony,color='red',linestyle='--',label='vicon',linewidth=1)
        plt.grid(True)
        ax.set_xticks([])
        #plt.ylim((-2,2))
        plt.ylabel('y(m)',fontsize=16)

        ax = fig.add_subplot(325)
        plt.plot(camerat,cameraz,color='blue',linestyle='--',label='camera',linewidth=1)
        # plt.plot(estimatet,estimatez,color='red',linestyle='--',label='estimate',linewidth=1)
        plt.plot(mhet1,mhez1,color='green',linestyle='--',label='mhe',linewidth=1)
        plt.plot(vicont,viconz,color='red',linestyle='--',label='vicon',linewidth=1)
        plt.grid(True)
        ax.set_xticks([])
        box=ax.get_position()
        ax.set_position([box.x0,box.y0,box.width,box.height*0.8])
        ax.legend()
        #plt.ylim((-0.5,2))
        plt.ylabel('z(m)',fontsize=16)





        ax = fig.add_subplot(322)
        plt.plot(camerat_vel,camerax_vel,color='blue',linestyle='--',label='camera',linewidth=1)
        # plt.plot(estimatet_vel,estimatex_vel,color='red',linestyle='--',label='estimate',linewidth=1)
        plt.plot(mhet1,mhex1_vel,color='green',linestyle='--',label='mhe',linewidth=1)
        plt.plot(vicont_vel,viconx_vel,color='red',linestyle='--',label='vicon',linewidth=1)
        plt.grid(True)
        ax.set_xticks([])
        # plt.ylim((-2,2))
        plt.ylabel('x(m/s)',fontsize=16)


        ax = fig.add_subplot(324)
        plt.plot(camerat_vel,cameray_vel,color='blue',linestyle='--',label='camera',linewidth=1)
        # plt.plot(estimatet_vel,estimatey_vel,color='red',linestyle='--',label='estimate',linewidth=1)
        plt.plot(mhet1,mhey1_vel,color='green',linestyle='--',label='mhe',linewidth=1)
        plt.plot(vicont_vel,vicony_vel,color='red',linestyle='--',label='vicon',linewidth=1)
        plt.grid(True)
        ax.set_xticks([])
        # plt.ylim((-2,2))
        plt.ylabel('y(m/s)',fontsize=16)

        ax = fig.add_subplot(326)
        plt.plot(camerat_vel,cameraz_vel,color='blue',linestyle='--',label='camera',linewidth=1)
        # plt.plot(estimatet_vel,estimatez_vel,color='red',linestyle='--',label='estimate',linewidth=1)
        plt.plot(mhet1,mhez1_vel,color='green',linestyle='--',label='mhe',linewidth=1)
        plt.plot(vicont_vel,viconz_vel,color='red',linestyle='--',label='vicon',linewidth=1)
        plt.grid(True)
        ax.set_xticks([])
        box=ax.get_position()
        ax.set_position([box.x0,box.y0,box.width,box.height*0.8])
        ax.legend()
        # plt.ylim((-0.5,2))
        plt.ylabel('z(m/s)',fontsize=16)



        plt.pause(0.01)  # 暂停一段时间
        plt.ioff()  # 关闭画图窗口

    else:
        pass



if __name__ == '__main__':
    global obt,obx,oby,obz,obt_vel,obx_vel,oby_vel,obz_vel
    global vit,viz,vix,viy,vit_vel,vix_vel,viy_vel,viz_vel
    global estt,estx,esty,estz,estt_vel,estx_vel,esty_vel,estz_vel
    global mhet,mhex,mhey,mhez,mhet_vel,mhex_vel,mhey_vel,mhez_vel
    global vel_pub,pos_pub


    obt = []
    obx = []
    oby = []
    obz = []
    vit = []
    vix = []
    viy = []
    viz = []
    estt = []
    estx = []
    esty = []
    estz = []
    mhet = []
    mhex = []
    mhey = []
    mhez = []

    obt_vel =  []
    obx_vel =  []
    oby_vel =  []
    obz_vel =  []
    vit_vel =  []
    vix_vel =  []
    viy_vel =  []
    viz_vel =  []
    estt_vel =  []
    estx_vel =  []
    esty_vel =  []
    estz_vel =  []
    mhet_vel =  []
    mhex_vel =  []
    mhey_vel =  []
    mhez_vel =  []

    try:
        rospy.init_node("mhe")
        pos_pub = rospy.Publisher('mhe_pos', PoseStamped, queue_size=1)
        
        vel_pub = rospy.Publisher('mhe_vel', TwistStamped, queue_size=1)
        rospy.loginfo("starting the node")
        mhe()
        plt.ion()  # 关闭画图窗口
        plt.show() #绘制完后不让窗口关闭
        while True:
            plot_real_time()
            time.sleep(0.001)
        rospy.spin()

    except KeyboardInterrupt:
        print("shutting down the node")
