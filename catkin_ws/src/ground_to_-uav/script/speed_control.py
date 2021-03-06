#!/usr/bin/env python
import time
import sys
import rospy
import math
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

wheel_track = 0.326/2 # half distance between two wheels

pub = []
odometry = Odometry()

def velocity_sp_cb(velocity_sp):

	delta_x = velocity_sp.x
	delta_y = velocity_sp.y
	print("velocity_sp x,y=(" + str(delta_x) + ", " +str(delta_y)+")")

	# calculate current orientation of the car
	x = odometry.pose.pose.orientation.x
	y = odometry.pose.pose.orientation.y
	z = odometry.pose.pose.orientation.z
	w = odometry.pose.pose.orientation.w

	current_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
	print("position xy:"+ str(odometry.pose.pose.position.x) + ", " + str(odometry.pose.pose.position.y))
	print("xyzw: " + str(x) +  ", " + str(y) +  ", " + str(z) +  ", " + str(w))
	print("current yaw: " + str(current_yaw))
	# calculate the speed of the car by projection of speed
	linear = delta_x * math.cos(current_yaw) + delta_y * math.sin(current_yaw)
	angular = -delta_x * math.sin(current_yaw) + delta_y * math.cos(current_yaw)# this is the speed perpendicular to linear, not actually angular
	angular = angular/wheel_track # this is actual angular
	# regularize the speed less than 1, in case it is too large

	print(linear)
	print(angular) 
	twist = Twist()
	twist.linear.x = linear*0.2
	twist.angular.z = angular*0.2

	max_linear_v = 0.5
	max_angular_v = 0.5

	if(abs(twist.linear.x) > max_linear_v):
		twist.linear.x =  max_linear_v * twist.linear.x / abs(twist.linear.x)

	if(abs(twist.angular.z) > max_angular_v):
		twist.angular.z =  max_angular_v * twist.angular.z / abs(twist.angular.z)

	# publish the speed
	print("linear:", twist.linear.x)
	print("angular", twist.angular.z)
	pub.publish(twist)


def odomCallback(odom):
	global odometry
	odometry = odom
	

def controller():   
	rospy.init_node('controller', anonymous = True)
	rospy.Subscriber("/car/odom", Odometry, odomCallback)
	rospy.Subscriber("/car/velocity_setpoint", Point, velocity_sp_cb)
	global pub
	pub = rospy.Publisher('/smoother_cmd_vel', Twist,queue_size=10)

	rospy.spin()

if __name__ == '__main__':
	controller()