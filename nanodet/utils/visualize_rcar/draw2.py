import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scipy.signal

data1 =np.loadtxt("/home/hjk/big_aruco/3_new/nano.txt")
data2 =np.loadtxt("/home/hjk/big_aruco/3_new/aruco.txt")
data3 =np.loadtxt("/home/hjk/big_aruco/3_new/vicon.txt")


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

nanot[:]= [x - t0 for x in nanot] 
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


arut[:]= [x - t0 for x in arut] 
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

vicont[:]= [x - t0 for x in vicont] 
viconx[:]= [100*(x - tx) for x in viconx] 
vicony[:]= [100*(x - ty) for x in vicony]
viconz[:]= [100*(x - tz) for x in viconz]  



# nanox = scipy.signal.savgol_filter(nanox,9,3)
# nanoy = scipy.signal.savgol_filter(nanoy,9,3)
# nanoz = scipy.signal.savgol_filter(nanoz,9,3)


# arux = scipy.signal.savgol_filter(arux,9,3)
# aruy = scipy.signal.savgol_filter(aruy,9,3)
# aruz = scipy.signal.savgol_filter(aruz,9,3)


# ax.plot3D(nanox,nanoy,nanoz,'red')
# ax.plot3D(arux,aruy,aruz,'blue')
# ax.plot3D(data3[:,0],data3[:,1],data3[:,2],'green')


plt.subplot(311)
plt.plot(arut,arux,'r-',label='aruco')
plt.plot(nanot,nanox,'b-',label='nanodet')
plt.plot(vicont,viconx,'g-',label='vicon')

plt.axis('tight')
plt.xlabel('time')
plt.ylabel('x/cm')
plt.title('ENU Coordinate System:x')

plt.subplot(312)
plt.plot(arut,aruy,'r-',label='aruco')
plt.plot(nanot,nanoy,'b-',label='nanodet')
plt.plot(vicont,vicony,'g-',label='vicon')

plt.axis('tight')
plt.xlabel('time')
plt.ylabel('y/cm')
plt.title('ENU Coordinate System:y')

plt.subplot(313)
plt.plot(arut,aruz,'r-',label='aruco')
plt.plot(nanot,nanoz,'b-',label='nanodet')
plt.plot(vicont,viconz,'g-',label='vicon')
plt.legend()
plt.grid(True)
plt.axis('tight')
plt.xlabel('time')
plt.ylabel('z/cm')
plt.title('ENU Coordinate System:z')
plt.tight_layout()

plt.show()