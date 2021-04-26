#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

namespace UWB_messages{
    struct UwbMessage{
        ros::Time systemTime;
        float ax;
        float ay;
        float az;
        float w;
        float x;
        float y;
        float z;
    };

    template<typename T>
    std::string convertToString(const T& message)
    {
        std::string s(sizeof(T), 0);
        memcpy((void*)(s.c_str()), (void*)(&message), sizeof(T));
        return s;
    }

    template<typename T>
    T convertToMessage(const std::vector<unsigned char>& s)
    {
        T message;
        memcpy((void*)(&message), (void*)(s.data()), sizeof(T));
        return message;
    }

}



UWB_messages::UwbMessage imu_msg;

void pose_data_sub_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_msg.systemTime = msg->header.stamp;
    imu_msg.ax = msg->linear_acceleration.x;
    imu_msg.ay = msg->linear_acceleration.y;
    imu_msg.az = msg->linear_acceleration.z;
    imu_msg.x = msg->orientation.x;
    imu_msg.y = msg->orientation.y;
    imu_msg.z = msg->orientation.z;
    imu_msg.w = msg->orientation.w;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwb_send");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    ros::Subscriber pose_data_sub = nh.subscribe<sensor_msgs::Imu>("/control_pose",1,pose_data_sub_callback);
    ros::Publisher uwb_data_pub = nh.advertise<std_msgs::String>("/nlink_linktrack_data_transmission", 1);
    std_msgs::String uwb_msg;

    while (ros::ok())
    {
        uwb_msg.data = UWB_messages::convertToString<UWB_messages::UwbMessage>(imu_msg);
        uwb_data_pub.publish(uwb_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}