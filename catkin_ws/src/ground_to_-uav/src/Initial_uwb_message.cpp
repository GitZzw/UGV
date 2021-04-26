#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <pthread.h>
#include <string>
#include "MvCameraControl.h"
#include "PoseCalculator.h"
#include "PoseMean.h"
#include "StampedQueue.h"
#include "UWBMessageQueue.h"
#include "TimeConverter.h"

#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include "UWB_message_type.h"
#include <nlink_parser/LinktrackNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNode0.h>
#include "geometry_msgs/Vector3Stamped.h"
#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>


StampedQueue<geometry_msgs::Vector3Stamped> g_servoPositionQueue;
StampedQueue<geometry_msgs::Vector3Stamped> g_servoVelocityQueue;

ros::Subscriber observe_sub;
geometry_msgs::Vector3Stamped servo_position_msg, servo_velocity_msg;

ros::Publisher uwb_data_local_pub;

ros::Publisher cameara_servo_detect_pub;
int count = 0;

bool calculatePublishPose(float vx, float vy, float vz, ros::Time stamp)
{
    // ros::spinOnce();
    bool initial_good_position = false;
    if(!ros::param::has("initial_good_position"))
    {
        ROS_ERROR("ros::param initial_good_position DOES NOT EXIST!!");
        return false;
    }
    ros::param::get("initial_good_position", initial_good_position);


    tf::Transform world_to_base_tf;
    std::vector<double> world_to_base_position(3,0), world_to_base_orientation({0,0,0,1});

    if( !ros::param::get("world_to_base_position", world_to_base_position) ||
        !ros::param::get("world_to_base_orientation", world_to_base_orientation) )
    {
        ros::param::set("initial_good_position", false);
        ROS_WARN_THROTTLE(2, "ros::param world_to_base_position, world_to_base_orientation NOT SET!, use (0,0,0) (0,0,0,1)");
    }
    tf::Quaternion world_to_base_q(world_to_base_orientation[0],world_to_base_orientation[1], world_to_base_orientation[2],world_to_base_orientation[3]);
    tf::Vector3 world_to_base_p(world_to_base_position[0],world_to_base_position[1],world_to_base_position[2]);
    world_to_base_tf = tf::Transform(world_to_base_q, world_to_base_p);

    geometry_msgs::Vector3Stamped servo_position_msg;
    if(!g_servoPositionQueue.getMessage(stamp, servo_position_msg))
    {
        ROS_ERROR("getStampedMsgFromQueue FAILED! g_servoPositionQueue");
        return false;
    }

    // ROS_INFO_STREAM("image stamp is " << stamp);

    // ROS_INFO_STREAM("servo back stamp is " << g_servoPositionQueue.q.back().header.stamp);

    // ROS_INFO_STREAM("servo front stamp is "<< g_servoPositionQueue.q.front().header.stamp);

    // ROS_INFO_STREAM("servo find stamp is " << servo_position_msg.header.stamp);


    UWB_messages::ObservationMessage obs_msg;
    obs_msg.systemTime = 0;
    obs_msg.servoControlMethod = 6;
    obs_msg.world_to_base = UWB_messages::Pose(world_to_base_tf);
    obs_msg.panTiltAngle.theta1 = servo_position_msg.vector.x;
    obs_msg.panTiltAngle.theta2 = servo_position_msg.vector.y;
    obs_msg.panTiltVelocity.theta1 = servo_velocity_msg.vector.x;
    obs_msg.panTiltVelocity.theta2 = servo_velocity_msg.vector.y;
    obs_msg.observedState[0].id = 0;

    std::vector<double> tvec;
    tvec.push_back(vx);
    tvec.push_back(vy);
    tvec.push_back(vz);

    obs_msg.observedState[0].pose = UWB_messages::Pose(tvec);  //uav_to_camera
    // ROS_INFO_STREAM(obs_msg);
    std_msgs::String uwb_obs_msg;
    uwb_obs_msg.data = UWB_messages::convertToString<UWB_messages::ObservationMessage>(obs_msg);
    uwb_data_local_pub.publish(uwb_obs_msg);

    geometry_msgs::PoseStamped cameara_servo_detect_msg;
    cameara_servo_detect_msg.header.stamp = stamp;

    cameara_servo_detect_msg.pose = obs_msg.get_geometry_pose_world();
    cameara_servo_detect_pub.publish(cameara_servo_detect_msg);
}


void servo_position_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    // servo_position_msg = *msg;
    g_servoPositionQueue.addMessage(*msg);
}

void servo_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    // servo_velocity_msg = *msg;
    g_servoVelocityQueue.addMessage(*msg);
}


void observe_CB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    
    // ros::Time start_time = ros::Time::now(), finish_time;
    calculatePublishPose(msg->vector.x,msg->vector.y,msg->vector.z,msg->header.stamp);
    // finish_time = ros::Time::now();
    // ROS_INFO_STREAM("calculatePublishPose Hz: " << 1/(finish_time-start_time).toSec());
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "servo_controller");
    ros::NodeHandle nh;
    observe_sub = nh.subscribe("/camera_detect2", 1, observe_CB);
    uwb_data_local_pub = nh.advertise<std_msgs::String>("/UWB_local_message", 2);
    cameara_servo_detect_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera_servo_detect", 1);
    ros::Subscriber servo_position_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/servo/position",10,servo_position_callback);
    ros::Subscriber servo_velocity_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/servo/velocity",10,servo_velocity_callback); 
    ros::spin();

    return 0;
}
