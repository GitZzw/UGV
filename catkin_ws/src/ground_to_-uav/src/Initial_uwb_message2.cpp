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
#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

const bool publish_image_topic_flag(false);
image_transport::Subscriber image_sub;

ros::Publisher pose_pub;
ros::Publisher pose_raw_pub;
ros::Publisher pose_pub0;
ros::Publisher pose_raw_pub0;
ros::Publisher pose_pub1;
ros::Publisher pose_raw_pub1;
ros::Publisher pose_pub2;
ros::Publisher pose_raw_pub2;
ros::Publisher pose_pub6;
ros::Publisher pose_raw_pub6;

ros::Publisher uwb_data_pub;
ros::Publisher uwb_data_local_pub;


TimeConverter g_timeConverter;
StampedQueue<geometry_msgs::Vector3Stamped> g_servoPositionQueue;
StampedQueue<geometry_msgs::Vector3Stamped> g_servoVelocityQueue;
UWBMessageQueue<UWB_messages::ObservationMessage> g_UWBMsgQueue;
PoseMean g_worldToBasePoseMean;
PoseCalculator g_poseCalculator;


double image_delay_ms = -1;
double camera_frame_rate;



bool calculatePublishPose(cv::Mat& image, int pnp_method_flag)
{
    int servo_tracking_status;
    cv::Mat rvec, tvec;
    g_poseCalculator.clearShowInfo();
    if(g_poseCalculator.calculate(image, rvec, tvec, pnp_method_flag))
    {
        //UWB message !!
        ros::spinOnce();
        bool initial_good_position = false;
        if(!ros::param::has("initial_good_position"))
        {
            ROS_ERROR("ros::param initial_good_position DOES NOT EXIST!!");
            return false;
        }
        ros::param::get("initial_good_position", initial_good_position);
        // ros::param::get("image_delay_ms", image_delay_ms);
        if( !ros::param::get("servo_tracking_status", servo_tracking_status))
        {
            servo_tracking_status = -1;
            ROS_WARN_THROTTLE(2, "ros::param servo_tracking_status not set!");
        }
        g_poseCalculator.addShowString("servo_tracking_status " + std::to_string(servo_tracking_status));

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
        geometry_msgs::Vector3Stamped servo_position_msg, servo_velocity_msg;
        ros::Time true_capture_rostime = ros::Time::now() - ros::Duration(image_delay_ms/1000.0);
        if(!g_servoPositionQueue.getMessage(true_capture_rostime, servo_position_msg))
        {
            ROS_ERROR("getStampedMsgFromQueue FAILED! g_servoPositionQueue");
            return false;
        }
        if(!g_servoVelocityQueue.getMessage(true_capture_rostime, servo_velocity_msg))
        {
            ROS_ERROR("getStampedMsgFromQueue FAILED! g_servoVelocityQueue");
            return false;
        }


        UWB_messages::ObservationMessage obs_msg;
        obs_msg.systemTime = g_timeConverter.convert_system_time(true_capture_rostime);
        obs_msg.servoControlMethod = servo_tracking_status;
        obs_msg.world_to_base = UWB_messages::Pose(world_to_base_tf);
        obs_msg.panTiltAngle.theta1 = servo_position_msg.vector.x;
        obs_msg.panTiltAngle.theta2 = servo_position_msg.vector.y;
        obs_msg.panTiltVelocity.theta1 = servo_velocity_msg.vector.x;
        obs_msg.panTiltVelocity.theta2 = servo_velocity_msg.vector.y;
        obs_msg.observedState[0].id = 0;
        cv::Mat tvec_meter = tvec/1000.0;
        obs_msg.observedState[0].pose = UWB_messages::Pose(rvec, tvec_meter);
        ROS_INFO_STREAM(obs_msg);
        std_msgs::String uwb_obs_msg;
        uwb_obs_msg.data = UWB_messages::convertToString<UWB_messages::ObservationMessage>(obs_msg);
        uwb_data_local_pub.publish(uwb_obs_msg);
        //visualization code
        // static tf::TransformBroadcaster br;
        // br.sendTransform(tf::StampedTransform(obs_msg.world_to_base.to_tf_transform(), ros::Time::now(), "map", "base" + std::to_string(0)));
        // br.sendTransform(tf::StampedTransform(obs_msg.panTiltAngle.get_tf_transform_base_s1(), ros::Time::now(), "base" + std::to_string(0), "servo1_" + std::to_string(0)));
        // br.sendTransform(tf::StampedTransform(obs_msg.panTiltAngle.get_tf_transform_s1_s2(), ros::Time::now(), "servo1_" + std::to_string(0), "servo2_" + std::to_string(0)));
        // br.sendTransform(tf::StampedTransform(obs_msg.panTiltAngle.get_tf_transform_s2_cam(), ros::Time::now(), "servo2_" + std::to_string(0), "cam" + std::to_string(0)));
        // br.sendTransform(tf::StampedTransform(obs_msg.observedState[0].pose.to_tf_transform(), ros::Time::now(), "cam" + std::to_string(0), "drone_" + std::to_string(0)));
        geometry_msgs::PoseStamped pose_stamped_world;
        pose_stamped_world.header.stamp = ros::Time::now();
        pose_stamped_world.pose = obs_msg.get_geometry_pose_world();
        pose_pub.publish(pose_stamped_world);
        ROS_INFO_STREAM(pose_stamped_world);

        switch(pnp_method_flag)
        {
            case 0:
                pose_pub0.publish(pose_stamped_world);
                break;
            case 1:
                pose_pub1.publish(pose_stamped_world);
                break;
            case 2:
                pose_pub2.publish(pose_stamped_world);
                break;
            case 6:
                pose_pub6.publish(pose_stamped_world);
                break;
        }

        if(initial_good_position)
        {
            uwb_data_pub.publish(uwb_obs_msg);
        }
        else
        {
            UWB_messages::ObservationMessage obs_msg_outside;
            uint32_t true_capture_system_time = g_timeConverter.convert_system_time(true_capture_rostime);

            ROS_WARN_STREAM("true_capture_rostime " << true_capture_rostime);
            ROS_WARN_STREAM("true_capture_system_time " << true_capture_system_time);
            if(!g_UWBMsgQueue.getMessage(true_capture_system_time, obs_msg_outside))
            {
                ROS_ERROR("getUWBMsgFromQueue FAILED!");
                return false;
            }

            geometry_msgs::Vector3Stamped servo_position_msg;
            if(!g_servoPositionQueue.getMessage(true_capture_rostime, servo_position_msg))
            {
                ROS_ERROR("getStampedMsgFromQueue FAILED!");
                return false;
            }
            UWB_messages::PanTiltAngle panTiltAngle;
            panTiltAngle.theta1 = servo_position_msg.vector.x;
            panTiltAngle.theta2 = servo_position_msg.vector.y;

            cv::Mat tvec_meter = tvec/1000.0;
            UWB_messages::Pose cam_to_drone(rvec, tvec_meter);
            tf::Transform world_to_base_tf_temp;
            world_to_base_tf_temp = obs_msg_outside.get_tf_transform_world() * 
                                    (panTiltAngle.get_tf_transform() * cam_to_drone.to_tf_transform()).inverse();

            //test 
            UWB_messages::Pose world_to_base_pose_temp(world_to_base_tf_temp);
            std::cout << g_worldToBasePoseMean.getDataSize() <<  world_to_base_pose_temp << std::endl;

            Eigen::Vector3d world_to_base_position_eigen(world_to_base_tf_temp.getOrigin().getX(),world_to_base_tf_temp.getOrigin().getY(),world_to_base_tf_temp.getOrigin().getZ());
            g_worldToBasePoseMean.addPosition(world_to_base_position_eigen);
            Eigen::Vector4d world_to_base_orientation_eigen(world_to_base_tf_temp.getRotation().getX(),world_to_base_tf_temp.getRotation().getY(),world_to_base_tf_temp.getRotation().getZ(),world_to_base_tf_temp.getRotation().getW());
            g_worldToBasePoseMean.addOrientation(world_to_base_orientation_eigen);

            int calibration_pose_count;
            if(!ros::param::has("calibration_pose_count"))
            {
                ROS_ERROR("ros::param calibration_pose_count DOES NOT EXIST!!");
                return false;
            }
            ros::param::get("calibration_pose_count", calibration_pose_count);
            if(g_worldToBasePoseMean.getDataSize() >= calibration_pose_count)
            {
                Eigen::Vector3d world_to_base_position_averaged = g_worldToBasePoseMean.getPositionMean();
                Eigen::Vector4d world_to_base_orientation_averaged = g_worldToBasePoseMean.getOrientationMean();

                world_to_base_tf.setOrigin(tf::Vector3(world_to_base_position_averaged.x(),world_to_base_position_averaged.y(),world_to_base_position_averaged.z()));
                world_to_base_tf.setRotation(tf::Quaternion(world_to_base_orientation_averaged.x(),
                                                            world_to_base_orientation_averaged.y(),
                                                            world_to_base_orientation_averaged.z(),
                                                            world_to_base_orientation_averaged.w()));
                ROS_WARN_STREAM("Calibration complete!! " << calibration_pose_count);
                UWB_messages::Pose world_to_base_pose(world_to_base_tf);
                ROS_WARN_STREAM(world_to_base_pose);
                g_worldToBasePoseMean.clear();
                ros::param::set("initial_good_position", true);
                world_to_base_position = std::vector<double>({world_to_base_position_averaged.x(),world_to_base_position_averaged.y(),world_to_base_position_averaged.z()});
                world_to_base_orientation = std::vector<double>({world_to_base_orientation_averaged.x(),
                                                world_to_base_orientation_averaged.y(),
                                                world_to_base_orientation_averaged.z(),
                                                world_to_base_orientation_averaged.w()});
                ros::param::set("world_to_base_position", world_to_base_position);
                ros::param::set("world_to_base_orientation", world_to_base_orientation);
            }
        }
    }
    g_poseCalculator.visualize();
}

void UWB_frame3_callback(const nlink_parser::LinktrackNodeframe3::ConstPtr &msg)
{
    g_timeConverter.updateBias(msg->system_time);
}

void UWB_frame2_callback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
{
    g_timeConverter.updateBias(msg->system_time);
}

void UWB_frame0_callback(const nlink_parser::LinktrackNodeframe0::ConstPtr &msg)
{
    for(int i = 0; i < msg->nodes.size();i++)
    {
        if(msg->nodes[i].data.size() == sizeof(UWB_messages::ObservationMessage))
        {
            UWB_messages::ObservationMessage obs_msg_outside(UWB_messages::convertToMessage<UWB_messages::ObservationMessage>(msg->nodes[i].data));
            g_UWBMsgQueue.addMessage(obs_msg_outside);
        }
    }
}

void servo_position_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    g_servoPositionQueue.addMessage(*msg);
}

void servo_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    g_servoVelocityQueue.addMessage(*msg);
}


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        ros::Time start_time = ros::Time::now(), finish_time;
        calculatePublishPose(cv_ptr->image,0);
        calculatePublishPose(cv_ptr->image,1);
        calculatePublishPose(cv_ptr->image,2);
        calculatePublishPose(cv_ptr->image,6);
        finish_time = ros::Time::now();
        ROS_INFO_STREAM("calculatePublishPose Hz: " << 1/(finish_time-start_time).toSec());
        char key_board = cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pnp_MV_cam");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_sub = it.subscribe("/MVcam/image", 5, imageCb);
    if(!ros::param::get("image_delay_ms", image_delay_ms))
    {
        ROS_ERROR("ros::param::get(\"image_delay_ms\", image_delay_ms) FAILED!!! ");
        return 0;
    }

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    pose_raw_pub = nh.advertise<geometry_msgs::PoseStamped>("/cam_raw/pose", 1);
    pose_pub0 = nh.advertise<geometry_msgs::PoseStamped>("/cam/pose0", 1);
    pose_raw_pub0 = nh.advertise<geometry_msgs::PoseStamped>("/cam_raw/pose0", 1);
    pose_pub1 = nh.advertise<geometry_msgs::PoseStamped>("/cam/pose1", 1);
    pose_raw_pub1 = nh.advertise<geometry_msgs::PoseStamped>("/cam_raw/pose1", 1);
    pose_pub2 = nh.advertise<geometry_msgs::PoseStamped>("/cam/pose2", 1);
    pose_raw_pub2 = nh.advertise<geometry_msgs::PoseStamped>("/cam_raw/pose2", 1);
    pose_pub6 = nh.advertise<geometry_msgs::PoseStamped>("/cam/pose6", 1);
    pose_raw_pub6 = nh.advertise<geometry_msgs::PoseStamped>("/cam_raw/pose6", 1);

    uwb_data_pub = nh.advertise<std_msgs::String>("/nlink_linktrack_data_transmission", 2);
    uwb_data_local_pub = nh.advertise<std_msgs::String>("/UWB_local_message", 2);
    ros::Subscriber UWB_frame0_sub = nh.subscribe<nlink_parser::LinktrackNodeframe0>("/nlink_linktrack_nodeframe0",10,UWB_frame0_callback);
    ros::Subscriber UWB_frame2_sub = nh.subscribe<nlink_parser::LinktrackNodeframe2>("/nlink_linktrack_nodeframe2",1,UWB_frame2_callback);
    ros::Subscriber UWB_frame3_sub = nh.subscribe<nlink_parser::LinktrackNodeframe3>("/nlink_linktrack_nodeframe3",1,UWB_frame3_callback);
    ros::Subscriber servo_position_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/servo/position",10,servo_position_callback);
    ros::Subscriber servo_velocity_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/servo/position",10,servo_velocity_callback);

    int camera_id;     //identify camera id
    if(!ros::param::has("camera_id"))
    {
        ROS_ERROR("ros::param camera_id DOES NOT EXIST!!");
        return 0;
    }
    ros::param::get("camera_id", camera_id);

    std::string camera_para_path = ros::package::getPath("ground_to_UAV") + "/data/camera" + std::to_string(camera_id) + ".txt";
	std::ifstream para_file(camera_para_path);
    if (!para_file.is_open())
	{
		ROS_ERROR_STREAM("can not open file: "+ camera_para_path);
		return 0;
	}

    std::vector<double> distcoeffs(5);
    cv::Mat cameraMatrix;

    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    para_file >> cameraMatrix.at<double>(0, 0) >> cameraMatrix.at<double>(0, 2) >> cameraMatrix.at<double>(1, 1) 
            >> cameraMatrix.at<double>(1, 2) >> cameraMatrix.at<double>(2, 2)
            >>  distcoeffs[0] >> distcoeffs[1];
    para_file.close();
    g_poseCalculator.cameraMatrix = cameraMatrix;
    g_poseCalculator.distcoeffs = distcoeffs;

    ros::spin();

    return 0;
}
