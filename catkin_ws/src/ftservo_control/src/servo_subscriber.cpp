/**
 * 该例程产生tf数据，并计算、发布turtle2的速度指令
 */


#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "SCServo.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"

const double L1 = 0.10;
const double L2 = 0.15;
std::string servo_name;

void poseCallback(const geometry_msgs::Vector3 theta)
{
	// 创建tf的广播器
	static tf::TransformBroadcaster br;

	// 初始化tf数据
	tf::Transform transform;


	//BOTTOM SERVO 
	transform.setOrigin( tf::Vector3(0, 0, L1) );
	tf::Quaternion q;
	q.setRPY(0, 0, theta.z);
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "servo1"));
	
	
	//TOP SERVO 
	tf:: Transform transform2;
	//transform2.setOrigin( tf::Vector3(L2*sin(theta.y),L2*cos(theta.y)*cos(theta.z), L2*cos(theta.y)*sin(theta.z)));
	transform2.setOrigin( tf::Vector3(L2*sin(theta.y),0,L2*cos(theta.y)));
	tf::Quaternion q2;
	q2.setRPY(0,theta.y,0);
	transform2.setRotation(q2);

	// 广播world与海龟坐标系之间的tf数据e

	br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "servo1", "servo2"));

	ROS_INFO("Successfully received:servo1 [%f] degree,top servo[%f] degree",theta.z, theta.y);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
	ros::init(argc, argv, "servo_broadcaster");

	servo_name = "servo";

	// 订阅海龟的位姿话题
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe<geometry_msgs::Vector3>("servo/pose", 10, &poseCallback);

    // 循环等待回调函数
	ros::spin();

	return 0;
};




