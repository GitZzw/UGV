import rospy
import sys
from sensor_msgs.msg import Image
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import time

global count
count = 1328

def callback(data):
    global count
    img = np.ndarray(shape=(data.height, data.width, 1),
	               dtype=np.uint8, buffer=data.data)

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imshow('img',img_rgb)

    cv2.imwrite(str(count)+'.png',img_rgb)
    count  = count + 1
    cv2.waitKey(1)
    time.sleep(0.2)


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/MVcam/image", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    listener()
