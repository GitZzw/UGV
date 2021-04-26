import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq
import os
from mpl_toolkits.mplot3d import Axes3D
import random
import scipy.signal


def moving_average(interval, windowsize):
    window = np.ones(int(windowsize)) / float(windowsize)
    re = np.convolve(interval, window, 'same')
    return re


def new_curve(nano,aruco):
    n = 1
    re = []
    for i in range(n):
        re.append(aruco[i])
    for i in range(len(aruco)-n):
        
        if(aruco[i]==aruco[i+n]):
            re.append(nano[i])
        else:
            re.append(aruco[i])
    return re


data1 =np.loadtxt("/home/mm/zzw/camera_detect.txt")
data2 =np.loadtxt("/home/mm/zzw/vicon.txt")



nanot = data1[:,0]
nanox = data1[:,1]
nanoy = data1[:,2]
nanoz = data1[:,3]



intervel=(nanot[-1]-nanot[0])/25.8
t0 = data1[0,0]
nanot[:]= [(x - t0)/intervel for x in nanot]


newx = nanox
newy = nanoy
newz = nanoz



#newx = moving_average(newx,25)

#newy = moving_average(newy,25)

#newz = moving_average(newz,25)



lengtht = len(nanot)



vicont = data2[:,0]
viconx = data2[:,1]
vicony = data2[:,2]
viconz = data2[:,3]



t0 = data2[0,0]

vicont[:]= [(x - t0)/intervel for x in vicont] 


fig = plt.figure()
ax = fig.add_subplot(311)

plt.plot(nanot[0:lengtht],nanox[0:lengtht],color='lime',linestyle='-',label='UBO',linewidth=0.5)
plt.plot(vicont,viconx,color='red',linestyle='--',label='Vicon',linewidth=1)



plt.grid(True)
ax.set_xticks([])
plt.ylabel('x(cm)',fontsize=16)


ax = fig.add_subplot(312)
plt.plot(nanot[0:lengtht],nanoy[0:lengtht],color='lime',linestyle='-',label='UBO',linewidth=0.5)
plt.plot(vicont,vicony,color='red',linestyle='--',label='Vicon',linewidth=1)
plt.grid(True)
ax.set_xticks([])
plt.ylabel('y(cm)',fontsize=16)

ax = fig.add_subplot(313)
# ax.set(ylim=[-100,350])
plt.plot(nanot[0:lengtht],nanoz[0:lengtht],color='lime',linestyle='-',label='UBO',linewidth=0.5)
plt.plot(vicont,viconz,color='red',linestyle='--',label='Vicon',linewidth=1)

box=ax.get_position()
ax.set_position([box.x0,box.y0,box.width,box.height*0.8])
ax.legend(bbox_to_anchor=(0.257,0.105),ncol=4,loc="center left",fontsize=16)
plt.grid(True)
plt.xlabel('Time(Sec)',fontsize=16)
plt.ylabel('z(cm)',fontsize=16)
plt.subplots_adjust(0.091,0.071,0.991,0.989,0.025,0.025)
plt.show()

