/**
 * 该例程监听tf数据
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "SCServo.h"
#include <iostream>

int main(int argc, char** argv)
{
	// 初始化ROS节点
	ros::init(argc, argv, "listener");

    // 创建节点句柄
	ros::NodeHandle node;


	tf::TransformListener listener;

	ros::Rate rate(10.0);
	while (node.ok())
	{
		
		tf::StampedTransform transform2;
		try
		{
			listener.waitForTransform("/servo2", "/servo1",ros::Time(0), ros::Duration(3.0));
			listener.lookupTransform("/servo2","/servo1", ros::Time(0), transform2);
		}
		catch (tf::TransformException &ex) 
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}



		rate.sleep();
	}
	return 0;
};
