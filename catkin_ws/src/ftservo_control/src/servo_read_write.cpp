#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include "SCServo.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

#define BOTTOM_SERVO_ID (1)
#define TOP_SERVO_ID    (2)
#define HORIZONTAL_DEAD_ZONE_IN_DEGREE (5.0)
#define VIRTICAL_DEAD_ZONE_IN_DEGREE (3.0)

#define ACC (100)
SMSBL servo_serial;

ros::Publisher servo_position_pub;
ros::Publisher servo_velocity_pub;
geometry_msgs::Vector3Stamped servo_position_msg;
geometry_msgs::Vector3Stamped servo_velocity_msg;

void position_setpoint_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 6)
    {
        ROS_ERROR_STREAM("Servo position setpoint size should be 6 !! Inputsize: "<< msg->data.size());
        return;
    }
    double unit = 50.0*360.0/4096 / 180 * M_PI;  // rad/s
    double bottom_position = (msg->data[0]+180) * (4096.0/360.0);
    float bottom_velocity = msg->data[1]/unit;
    float bottom_acc = msg->data[2];
    double top_position = (msg->data[3]+180) * (4096.0/360.0);
    float top_velocity = msg->data[4]/unit;
    float top_acc = msg->data[5];
    servo_serial.WritePosEx(BOTTOM_SERVO_ID, int(bottom_position), int(bottom_velocity), int(bottom_acc));
    servo_serial.WritePosEx(TOP_SERVO_ID, int(top_position), int(top_velocity), int(top_acc));
}

void speed_setpoint_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size() != 2) 
    {
        ROS_ERROR_STREAM("Servo speed setpoint size should be 2 !! Inputsize: "<< msg->data.size());
        return;
    }
    double unit = 50.0*360.0/4096 / 180 * M_PI;  // rad/s
    if(!servo_serial.WriteSpe(1, int(msg->data[0]/unit), ACC))
        ROS_ERROR("WRITE SPEED ERROR");
    servo_serial.WriteSpe(2, int(msg->data[1]/unit), ACC);
    ROS_INFO("speed setpoint %d  %d", int(msg->data[0]/unit), int(msg->data[1]/unit));
}

void servo_pub_timer_callback(const ros::TimerEvent&)
{
    if(servo_serial.FeedBack(BOTTOM_SERVO_ID) != -1 && servo_serial.FeedBack(TOP_SERVO_ID) != -1)
    {
        servo_position_msg.vector.x = servo_serial.ReadPos(BOTTOM_SERVO_ID)*360.0/4096-180;
        servo_position_msg.vector.y = servo_serial.ReadPos(TOP_SERVO_ID)*360.0/4096-180;
        servo_position_msg.header.stamp = ros::Time::now();
        servo_position_pub.publish(servo_position_msg);

        servo_velocity_msg.vector.x = servo_serial.ReadSpeed(BOTTOM_SERVO_ID);
        servo_velocity_msg.vector.y = servo_serial.ReadSpeed(TOP_SERVO_ID);
        servo_velocity_msg.header.stamp = ros::Time::now();
        servo_velocity_pub.publish(servo_velocity_msg);

    }
    else
    {
        ROS_WARN_STREAM_THROTTLE(0.2, "servo_serial.FeedBack == -1");
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "servo_read_write");
    ros::NodeHandle nh;

    if(argc != 2){
        ROS_ERROR("argc != 2 error!");
        return 0;
	}
    // if(!servo_serial.begin(115200, argv[1])){
    if(!servo_serial.begin(1000000, argv[1])){
        ROS_ERROR_STREAM("Failed to init smsbl motor! " << argv[1]);
        return 0;
    }

    ros::Subscriber position_setpoint_sub = nh.subscribe<std_msgs::Float32MultiArray>("/servo/position_setpoint", 1, &position_setpoint_callback);
    ros::Subscriber speed_setpoint_sub = nh.subscribe<std_msgs::Float32MultiArray>("/servo/speed_setpoint", 1, &speed_setpoint_callback);
    servo_position_pub = nh.advertise<geometry_msgs::Vector3Stamped> ("/servo/position",10);
    servo_velocity_pub = nh.advertise<geometry_msgs::Vector3Stamped> ("/servo/velocity",10);

    double servo_pub_rate;
    if(!ros::param::get("servo_pub_rate", servo_pub_rate))
    {
        ROS_ERROR("ros::param::get(\"servo_pub_rate\", servo_pub_rate) FAILED!!! ");
        return 0;
    }
    ros::Timer servo_pub_timer = nh.createTimer(ros::Duration(1/servo_pub_rate), servo_pub_timer_callback);

	ros::spin();
    return 0;
}

