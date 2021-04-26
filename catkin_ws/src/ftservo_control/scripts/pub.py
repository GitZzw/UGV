#! /usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray
def servo_controller():
    pub_p = rospy.Publisher('/servo/position_setpoint', Float32MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	array = [0,100,30,0,100,30]
	msg = Float32MultiArray(data=array)	
        rospy.loginfo(msg)
	pub_p.publish(msg)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        servo_controller()
    except rospy.ROSInterruptException:
        pass
