from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import rospy
from cat.msg import zzw
import matplotlib.pyplot as plt
import threading

global nanox
global nanoy
global nanoz  
global flag

global arucox
global arucoy
global arucoz

nanox = 0
nanoy = 0
nanoz = 0
arucox = 0
arucoy = 0 
arucoz = 0
def arucocallback(data):
    #uav cordinates
    global flag 
    global arucox
    global arucoy
    global arucoz 
    # global deltax
    # global deltay 
    # global deltaz 

    arucox = data.pose.position.x
    arucoy = data.pose.position.y
    arucoz = data.pose.position.z

    if(flag == True):
        flag = False
        t2 = threading.Thread(target=plotrealtime)
        t2.start()


def plotrealtime():
    global arucox
    global arucoy
    global arucoz
    global nanox
    global nanoy
    global nanoz
    fflag =False
    i = 1
    plt.ion()   # 开启一个画图的窗口
    arux=[]  # 定义一个 x 轴的空列表用来接收动态的数据
    nax = []
    aruy=[]
    nay = []
    aruz=[]
    naz = []
    t=[]
    plt.figure(figsize=(12,16))
    while True: 
        # if(fflag or (len(arux)>=51 and len(aruy)>=51 and len(aruz)>=51 and len(t)>=51 and len(nax)>=51 and len(nay)>=51 and len(naz)>=51)):
        if(False):
            fflag = True
            parux = arux[len(arux)-50:]
            paruy = aruy[len(aruy)-50:]
            paruz = aruz[len(aruz)-50:]    
            pt = t[len(t)-50:]
            pnanox = nax[len(nax)-50:]
            pnanoy = nay[len(nay)-50:]
            pnanoz = naz[len(naz)-50:]
        else:
            parux = arux
            paruy = aruy
            paruz = aruz   
            pt = t
            pnanox = nax
            pnanoy = nay
            pnanoz = naz


        i = i + 0.1
        t.append(i)
        arux.append(arucox)  # 添加 i 到 x 轴的数据中
        aruy.append(arucoy)
        aruz.append(arucoz) # 添加 i 的平方到 y 轴的数据中
        nax.append(nanox)
        nay.append(nanoy)
        naz.append(nanoz)

        plt.clf()  # 清除之前画的图
        plt.subplot(311)
        plt.plot(pt,parux,'r.-',lw = 1,label='aruco')
        plt.plot(pt,pnanoz,'g.-',lw = 1,label='nanodet')
        plt.legend()
        plt.grid(True)
        plt.axis('tight')
        plt.xlabel('time')
        plt.ylabel('x/cm')
        plt.title('ENU Coordinate System:x')

        plt.subplot(312)
        plt.plot(pt,paruy,'r.-',lw = 1,label='aruco')
        plt.plot(pt,pnanox,'g.-',lw = 1,label='nanodet')
        plt.legend()
        plt.grid(True)
        plt.axis('tight')
        plt.xlabel('time')
        plt.ylabel('y/cm')
        plt.title('ENU Coordinate System:y')

        plt.subplot(313)
        plt.plot(pt,paruz,'r.-',lw = 1,label='aruco')
        plt.plot(pt,pnanoy,'g.-',lw = 1,label='nanodet')
        plt.legend()
        plt.grid(True)
        plt.axis('tight')
        plt.xlabel('time')
        plt.ylabel('z/cm')
        plt.title('ENU Coordinate System:z')
        plt.tight_layout()
        plt.pause(0.3)  # 暂停一秒
        plt.ioff()  # 关闭画图的窗口



def nanocallback(data):
    #xyz cordinates
    global nanox
    global nanoy
    global nanoz 
    nanox = -data.pose.orientation.x
    nanoy = -data.pose.orientation.y
    nanoz = data.pose.orientation.z



def compare():
    rospy.init_node('compare', anonymous=True)   
    rospy.Subscriber("aruco_detect/send_data_small", zzw, arucocallback)
    rospy.Subscriber("yolo_target_corner", PoseStamped, nanocallback)
    rospy.spin()


if __name__ == "__main__":
    global flag
    flag = True
    compare()
