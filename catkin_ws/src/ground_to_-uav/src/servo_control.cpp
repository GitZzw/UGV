#include <ros/ros.h>
#include "UWB_message_type.h"
#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nlink_parser/LinktrackNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include "std_msgs/Float32MultiArray.h"
#include <map>
#include "TimeConverter.h"

#define CAMERA_ANGLE_HORIZONTAL_DEGREE (25.0) //two sides
#define CAMERA_ANGLE_VERTICAL_DEGREE (25.0)
#define CAMERA_LOST_DANGER_DEGREE_BASE (10.0) //one side
#define SPEED_P (1)    
#define MAX_DRONE_SPEED (1.0) // m/s
#define ACC (150)
#define VEL (100)

ros::Publisher servo_position_setpoint_pub;
ros::Publisher servo_speed_setpoint_pub;
std::map<int, UWB_messages::ObservationMessage> otherObservations;
UWB_messages::ObservationMessage obs_msg_self;


TimeConverter g_timeConverter;
bool initial_good_position = false;
int servo_tracking_status = 6; // 1 for position, 2 for speed
int camera_frame_rate;

void UWB_msg_self_callback(const std_msgs::String::ConstPtr &msg)
{
    obs_msg_self = UWB_messages::convertToMessage<UWB_messages::ObservationMessage>(msg->data);
    
    double drone_to_cam_distance = obs_msg_self.observedState[0].pose.position.z;
    ROS_INFO_STREAM(drone_to_cam_distance);
    UWB_messages::PanTiltAngle angles_target;

    tf::Transform base_to_drone_tf = obs_msg_self.panTiltAngle.get_tf_transform()*obs_msg_self.observedState[0].pose.to_tf_transform();
    
    
    angles_target = UWB_messages::rotateTowards(base_to_drone_tf);

    
    double theta1_not_move_thre, theta2_not_move_thre,delta_theta1,delta_theta2;
    delta_theta1 = angles_target.theta1 - obs_msg_self.panTiltAngle.theta1;
    delta_theta2 = angles_target.theta2 - obs_msg_self.panTiltAngle.theta2;
    // theta1_not_move_thre = CAMERA_ANGLE_HORIZONTAL_DEGREE / 2 - 
    //                     CAMERA_LOST_DANGER_DEGREE_BASE/obs_msg_self.observedState[0].pose.position.x;
    // theta2_not_move_thre = CAMERA_ANGLE_VERTICAL_DEGREE / 2 - 
    //                     CAMERA_LOST_DANGER_DEGREE_BASE/obs_msg_self.observedState[0].pose.position.x;
    // double theta1_safe_thre, theta2_safe_thre;
    // theta1_safe_thre = CAMERA_ANGLE_HORIZONTAL_DEGREE / 2 - 5 - 
    //                     CAMERA_LOST_DANGER_DEGREE_BASE/obs_msg_self.observedState[0].pose.position.x;
    // theta2_safe_thre = CAMERA_ANGLE_VERTICAL_DEGREE / 2 - 5 -
    //                     CAMERA_LOST_DANGER_DEGREE_BASE/obs_msg_self.observedState[0].pose.position.x;


    // std::cout<<"1:"<<theta1_not_move_thre<<std::endl;
    // std::cout<<"2:"<<theta1_not_move_thre<<std::endl;

    if(servo_tracking_status == 6)
    {   
        std::vector<float> servo_sp_vec(6); // servo1: pos,vel,acc; servo2:pos,vel,acc

        servo_sp_vec[0] = angles_target.theta1;
        // if(std::abs(delta_theta1)<2.0){
        //     servo_sp_vec[0] = obs_msg_self.panTiltAngle.theta1;
        // }
        // else if(angles_target.theta1<-10.0){
        //     servo_sp_vec[0] = -10.0;
        // }
        // else if(angles_target.theta1>10.0){
        //     servo_sp_vec[0] = 10.0;
        // }
        // else{
        //     servo_sp_vec[0] = angles_target.theta1;
        // }
        // if(std::abs(delta_theta2)<2.0){
        //     servo_sp_vec[3] = obs_msg_self.panTiltAngle.theta2;
        // }
        // else if(angles_target.theta2>0.0){
        //     servo_sp_vec[3] = 0.0; 
        // }
        // else if(angles_target.theta2<-10){
        //     servo_sp_vec[3] = -10; 
        // }
        // else{
        //     servo_sp_vec[3] = angles_target.theta2;
        // }
        servo_sp_vec[3] = angles_target.theta2;
        servo_sp_vec[1] = VEL;
        servo_sp_vec[4] = VEL;
        servo_sp_vec[2] = ACC/drone_to_cam_distance/drone_to_cam_distance;
        servo_sp_vec[5] = ACC/drone_to_cam_distance/drone_to_cam_distance;
        // servo_sp_vec[2] = 150;
        // servo_sp_vec[5] = 150;
        std_msgs::Float32MultiArray servo_setpoint;
        servo_setpoint.data = servo_sp_vec;
        // std::cout<<angles_target.theta1<<" "<<angles_target.theta2<<std::endl;
        servo_position_setpoint_pub.publish(servo_setpoint);

    }
       
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "servo_control");
    ros::NodeHandle nh;
    servo_position_setpoint_pub = nh.advertise<std_msgs::Float32MultiArray>("/servo/position_setpoint", 1);
    servo_speed_setpoint_pub = nh.advertise<std_msgs::Float32MultiArray>("/servo/speed_setpoint", 1);

    ros::Subscriber UWB_msg_self_sub = nh.subscribe<std_msgs::String>("/UWB_local_message", 1, UWB_msg_self_callback);


    ros::spin();
    return 0;
}