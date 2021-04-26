import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq
import os
from mpl_toolkits.mplot3d import Axes3D
import random
import scipy.signal

def plot_vel():
    data1 =np.loadtxt("/home/mm/zzw/data_analysis/observe_vel.txt")
    data2 =np.loadtxt("/home/mm/zzw/data_analysis/estimate_vel.txt")
    data3 =np.loadtxt("/home/mm/zzw/data_analysis/mhe_vel.txt")
    data4 = np.loadtxt("/home/mm/zzw/data_analysis/vicon_vel.txt")


    camerat = data1[:,0]
    camerax = data1[:,1]
    cameray = data1[:,2]
    cameraz = data1[:,3]
    intervel=(camerat[-1]-camerat[0])/60 ## time length
    t0 = data1[0,0]
    camerat[:]= [(x - t0)/intervel for x in camerat]


    lengtht = len(camerat)

    estimatet = data2[:,0]
    estimatex = data2[:,1]
    estimatey = data2[:,2]
    estimatez = data2[:,3]
    t0 = data2[0,0]
    estimatet[:]= [(x - t0)/intervel for x in estimatet] 

    mhet = data3[:,0]
    mhex = data3[:,1]
    mhey = data3[:,2]
    mhez = data3[:,3]
    t0 = data3[0,0]
    mhet[:] = [(x - t0)/intervel for x in mhet]


    vicont = data4[:,0]
    viconx = data4[:,1]
    vicony = data4[:,2]
    viconz = data4[:,3]
    t0 = data4[0,0]
    vicont[:] = [(x - t0)/intervel for x in vicont]
    # vicont[:] = [ x for x in vicont]

    fig = plt.figure()
    ax = fig.add_subplot(311)
    plt.plot(camerat[0:lengtht],camerax[0:lengtht],color='blue',linestyle='--',label='camera',linewidth=1)
    plt.plot(estimatet,estimatex,color='red',linestyle='--',label='estimate',linewidth=1)
    plt.plot(mhet,mhex,color='green',linestyle='--',label='mhe',linewidth=1)
    plt.plot(vicont,viconx,color='peru',linestyle='--',label='vicon',linewidth=1)
    plt.grid(True)
    ax.set_xticks([])
    plt.ylabel('x(m)',fontsize=16)



    ax = fig.add_subplot(312)
    plt.plot(camerat[0:lengtht],cameray[0:lengtht],color='blue',linestyle='--',label='camera',linewidth=1)
    plt.plot(estimatet,estimatey,color='red',linestyle='--',label='estimate',linewidth=1)
    plt.plot(mhet,mhey,color='green',linestyle='--',label='mhe',linewidth=1)
    plt.plot(vicont,vicony,color='peru',linestyle='--',label='vicon',linewidth=1)
    plt.grid(True)
    ax.set_xticks([])
    plt.ylabel('y(m)',fontsize=16)


    ax = fig.add_subplot(313)
    plt.plot(camerat[0:lengtht],cameraz[0:lengtht],color='blue',linestyle='--',label='camera',linewidth=1)
    plt.plot(estimatet,estimatez,color='red',linestyle='--',label='estimate',linewidth=1)
    plt.plot(mhet,mhez,color='green',linestyle='--',label='mhe',linewidth=1)
    plt.plot(vicont,viconz,color='peru',linestyle='--',label='vicon',linewidth=1)



    box=ax.get_position()
    ax.set_position([box.x0,box.y0,box.width,box.height*0.8])
    ax.legend()
    plt.grid(True)
    plt.xlabel('Time(Sec)',fontsize=16)
    plt.ylabel('z(m)',fontsize=16)
    plt.show()



def plot_pos():

    data1 =np.loadtxt("/home/mm/zzw/data_analysis/observe.txt")
    data2 =np.loadtxt("/home/mm/zzw/data_analysis/estimate.txt")
    data3 =np.loadtxt("/home/mm/zzw/data_analysis/mhe.txt")
    data4 = np.loadtxt("/home/mm/zzw/data_analysis/vicon.txt")

    camerat = data1[:,0]
    camerax = data1[:,1]
    cameray = data1[:,2]
    cameraz = data1[:,3]
    intervel=(camerat[-1]-camerat[0])/60 ## time length
    t0 = data1[0,0]
    camerat[:]= [(x - t0)/intervel for x in camerat]


    lengtht = len(camerat)

    estimatet = data2[:,0]
    estimatex = data2[:,1]
    estimatey = data2[:,2]
    estimatez = data2[:,3]
    t0 = data2[0,0]
    estimatet[:]= [(x - t0)/intervel for x in estimatet] 

    mhet = data3[:,0]
    mhex = data3[:,1]
    mhey = data3[:,2]
    mhez = data3[:,3]
    t0 = data3[0,0]
    mhet[:] = [(x - t0)/intervel for x in mhet]


    vicont = data4[:,0]
    viconx = data4[:,1]
    vicony = data4[:,2]
    viconz = data4[:,3]
    t0 = data4[0,0]
    vicont[:] = [(x - t0)/intervel for x in vicont]
    # vicont[:] = [ x for x in vicont]

    fig = plt.figure()
    ax = fig.add_subplot(311)
    plt.plot(camerat[0:lengtht],camerax[0:lengtht],color='blue',linestyle='--',label='camera',linewidth=1)
    plt.plot(estimatet,estimatex,color='red',linestyle='--',label='estimate',linewidth=1)
    plt.plot(mhet,mhex,color='green',linestyle='--',label='mhe',linewidth=1)
    plt.plot(vicont,viconx,color='peru',linestyle='--',label='vicon',linewidth=1)
    plt.grid(True)
    ax.set_xticks([])
    plt.ylabel('x(m)',fontsize=16)



    ax = fig.add_subplot(312)
    plt.plot(camerat[0:lengtht],cameray[0:lengtht],color='blue',linestyle='--',label='camera',linewidth=1)
    plt.plot(estimatet,estimatey,color='red',linestyle='--',label='estimate',linewidth=1)
    plt.plot(mhet,mhey,color='green',linestyle='--',label='mhe',linewidth=1)
    plt.plot(vicont,vicony,color='peru',linestyle='--',label='vicon',linewidth=1)
    plt.grid(True)
    ax.set_xticks([])
    plt.ylabel('y(m)',fontsize=16)


    ax = fig.add_subplot(313)
    plt.plot(camerat[0:lengtht],cameraz[0:lengtht],color='blue',linestyle='--',label='camera',linewidth=1)
    plt.plot(estimatet,estimatez,color='red',linestyle='--',label='estimate',linewidth=1)
    plt.plot(mhet,mhez,color='green',linestyle='--',label='mhe',linewidth=1)
    plt.plot(vicont,viconz,color='peru',linestyle='--',label='vicon',linewidth=1)



    box=ax.get_position()
    ax.set_position([box.x0,box.y0,box.width,box.height*0.8])
    ax.legend()
    plt.grid(True)
    plt.xlabel('Time(Sec)',fontsize=16)
    plt.ylabel('z(m)',fontsize=16)
    # plt.show()



if __name__ == '__main__':
    
    plot_pos()
    plot_vel()