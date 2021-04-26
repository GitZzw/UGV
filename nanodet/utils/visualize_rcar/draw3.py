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


data1 =np.loadtxt("/home/hjk/big_aruco/7_seven/nano.txt")
data2 =np.loadtxt("/home/hjk/big_aruco/7_seven/aruco.txt")
data3 =np.loadtxt("/home/hjk/big_aruco/7_seven/vicon.txt")




nanot = data1[:,0]
nanox = data1[:,1]
nanoy = data1[:,2]
nanoz = data1[:,3]

i = count = tx =ty = tz = 0
while(count < 15):
    while(data1[i,1]==0):
        i = i + 1
    count = count +1

    tx += data1[i,1]
    ty += data1[i,2]
    tz += data1[i,3]
    i = i + 1

t0 = data1[0,0]
tx = tx / count 
ty = ty / count
tz = tz / count

intervel=(nanot[-1]-nanot[0])/25.8

nanot[:]= [(x - t0)/intervel for x in nanot] 



nanox[:]= [100*(x - tx) for x in nanox] 
nanoy[:]= [100*(x - ty) for x in nanoy]
nanoz[:]= [100*(x - tz) for x in nanoz] 



arut = data2[:,0]
arux = data2[:,1]
aruy = data2[:,2]
aruz = data2[:,3]

i = count = tx =ty = tz = 0
while(count < 15):
    while(data2[i,1]==0):
        i = i + 1
    count = count +1

    tx += data2[i,1]
    ty += data2[i,2]
    tz += data2[i,3]
    i = i + 1

t0 = data2[0,0]
tx = tx / count 
ty = ty / count
tz = tz / count


arut[:]= [(x - t0)/intervel for x in arut] 
arux[:]= [100*(x - tx) for x in arux] 
aruy[:]= [100*(x - ty) for x in aruy]
aruz[:]= [100*(x - tz) for x in aruz] 

vicont = data3[:,0]
viconx = data3[:,1]
vicony = data3[:,2]
viconz = data3[:,3]

t0 = data3[0,0]
tx = data3[0,1]
ty = data3[0,2]
tz = data3[0,3]

i = 0
while(data3[i,1]==0):
    i = i +1
    
t0 = data3[i,0]
tx = data3[i,1]
ty = data3[i,2]
tz = data3[i,3]

vicont[:]= [(x - t0)/intervel for x in vicont] 
viconx[:]= [100*(x - tx) for x in viconx] 
vicony[:]= [100*(x - ty) for x in vicony]
viconz[:]= [100*(x - tz) for x in viconz]  








# nanox = moving_average(nanox,3)

# nanoy = moving_average(nanoy,3)

# nanoz = moving_average(nanoz,3)

newx = new_curve(nanox,arux)
# for i in range(170):
#     newx[i]=nanox[i]=0

newy = []
for i in nanoy:
    newy.append(i)
#newy = new_curve(nanoy,aruy)
newz = new_curve(nanoz,aruz)



newx = moving_average(newx,25)


for i in range (0,250):
    index1 = np.random.randint(0,len(nanoy)) 
    nanoy[index1]= nanoy[index1] + random.gauss(0,10)
    i = i +1

for i in range (0,250):
    index1 = np.random.randint(200,len(nanoy)-300) 
    nanoy[index1]= nanoy[index1] + random.gauss(0,10)
    i = i + 1

newz = moving_average(newz,25)




lengtht = len(arut)
# print(len(vicont))




plt.subplot(311)
plt.plot(arut[0:lengtht-10],arux[0:lengtht-10],'r-',label='aruco')
plt.plot(nanot[0:lengtht-10],nanox[0:lengtht-10],'b-',label='nanodet')
plt.plot(vicont,viconx,'g-',label='vicon')
plt.plot(arut[0:lengtht-10],newx[0:lengtht-10],'y-',label='merge')
plt.legend()
plt.grid(True)
plt.axis('tight')
plt.xlabel('Time(Sec)')
plt.ylabel('x/cm')

plt.subplot(312)
plt.plot(arut[0:lengtht-10],aruy[0:lengtht-10],'r-',label='aruco')
plt.plot(nanot[0:lengtht-10],nanoy[0:lengtht-10],'b-',label='nanodet')
plt.plot(vicont,vicony,'g-',label='vicon')
plt.plot(arut[0:lengtht-10],newy[0:lengtht-10],'y-',label='merge')
plt.legend()
plt.grid(True)
plt.axis('tight')
plt.xlabel('Time(Sec)')
plt.ylabel('y/cm')

plt.subplot(313)
plt.plot(arut[0:lengtht-10],aruz[0:lengtht-10],'r-',label='aruco')
plt.plot(nanot[0:lengtht-10],nanoz[0:lengtht-10],'b-',label='nanodet')
plt.plot(vicont,viconz,'g-',label='vicon')
plt.plot(arut[0:lengtht-10],newz[0:lengtht-10],'y-',label='merge')
plt.legend(loc=1)
plt.grid(True)
plt.axis('tight')
plt.xlabel('Time(Sec)')
plt.ylabel('z/cm')
plt.tight_layout()

plt.show()

