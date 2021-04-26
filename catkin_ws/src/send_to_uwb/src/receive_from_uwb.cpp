#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <nlink_parser/LinktrackNodeframe0.h>
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


sensor_msgs::Imu imu_msg;
void uwb_data_sub_callback(const nlink_parser::LinktrackNodeframe0::ConstPtr& msg)
{
    for(int i = 0; i < msg->nodes.size();i++)
    {
        if(msg->nodes[i].data.size() == sizeof(UWB_messages::UwbMessage))
        {
            UWB_messages::UwbMessage uwb_msg(UWB_messages::convertToMessage<UWB_messages::UwbMessage>(msg->nodes[i].data));
            imu_msg.linear_acceleration.x = uwb_msg.ax;
            imu_msg.linear_acceleration.y = uwb_msg.ay;
            imu_msg.linear_acceleration.z = uwb_msg.az;
            imu_msg.orientation.w = uwb_msg.w;
            imu_msg.orientation.x = uwb_msg.x;
            imu_msg.orientation.y = uwb_msg.y;
            imu_msg.orientation.z = uwb_msg.z;
            imu_msg.header.stamp = uwb_msg.systemTime;
            // std::cout<< ros::Time::now() <<std::endl;
            // std::cout<< ros::Time::now()-uwb_msg.systemTime <<std::endl;
        }
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "uwb_receive");
    ros::NodeHandle nh;
    ros::Publisher imu_data_pub = nh.advertise<sensor_msgs::Imu>("/uwb_imu", 1);
    ros::Rate loop_rate(100);
    ros::Subscriber imu_data_sub = nh.subscribe<nlink_parser::LinktrackNodeframe0>("/nlink_linktrack_nodeframe0",1,uwb_data_sub_callback);
    while (ros::ok()){
        imu_data_pub.publish(imu_msg);
        ros::spinOnce();  
        loop_rate.sleep();
    }

    return 0;
}
