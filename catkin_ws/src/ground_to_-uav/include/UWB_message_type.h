#ifndef UWB_MESSAGE_TYPE_H
#define UWB_MESSAGE_TYPE_H

#include <stdint.h>
#include <iostream>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <tf/tf.h>

#define h1 (0.04f)          //not important
#define h2 (0.05f)          //not important
#define h3 (0.06805f)
#define h4 (0.01f)          //need more test

#define BOARD_CENTER_X (-0.15f)
#define BOARD_CENTER_Y (0.0f)
#define BOARD_CENTER_Z (0.0f)

#define MAX_DRONE_NUMBER 1

#define UWB_DATA_TYPE float

#pragma pack(push)
#pragma pack(1)
namespace UWB_messages
{

    tf::Quaternion cvtToTFQuaternion(std::vector<double> rvec)
    {
        tf::Quaternion q;
        double theta = sqrt( rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
        tf::Vector3 axis = tf::Vector3( rvec[0]/theta, rvec[1]/theta, rvec[2]/theta);
        q.setRotation(axis,theta);
        return q;
    }

    std::string displayRPY(tf::Quaternion q)
    {
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(q);
        mat.getEulerYPR(yaw, pitch, roll);
        return "DEG:\troll: " + std::to_string(roll/M_PI*180) + " pitch: " + std::to_string(pitch/M_PI*180) + " yaw: " + std::to_string(yaw/M_PI*180);
    }

    struct Vector3
    {
        UWB_DATA_TYPE x;
        UWB_DATA_TYPE y;
        UWB_DATA_TYPE z;
        Vector3():x(0),y(0),z(0){}
        Vector3(UWB_DATA_TYPE x, UWB_DATA_TYPE y, UWB_DATA_TYPE z):x(x),y(y),z(z){}
        Vector3(geometry_msgs::Vector3 v):x(v.x),y(v.y),z(v.z){}
        Vector3(geometry_msgs::Point v):x(v.x),y(v.y),z(v.z){}

        friend std::ostream& operator<< (std::ostream &out, Vector3 &vector3)
        {
            out << "[x: " << vector3.x << ", y: "<< vector3.y << ", z: "<< vector3.z << "]" << std::endl;
            return out;
        }
        geometry_msgs::Vector3 to_geometry_vector3()
        {
            geometry_msgs::Vector3 vec3_msg;
            vec3_msg.x = x;
            vec3_msg.y = y;
            vec3_msg.z = z;
            return vec3_msg;
        }
        geometry_msgs::Point to_geometry_point()
        {
            geometry_msgs::Point point_msg;
            point_msg.x = x;
            point_msg.y = y;
            point_msg.z = z;
            return point_msg;
        }
    };

    struct Quaternion
    {
        UWB_DATA_TYPE w;
        UWB_DATA_TYPE x;
        UWB_DATA_TYPE y;
        UWB_DATA_TYPE z;
        Quaternion():w(0),x(0),y(0),z(0){}
        Quaternion(UWB_DATA_TYPE w, UWB_DATA_TYPE x, UWB_DATA_TYPE y, UWB_DATA_TYPE z):w(w),x(x),y(y),z(z){}
        Quaternion(geometry_msgs::Quaternion q):w(q.w),x(q.x),y(q.y),z(q.z){}
        Quaternion(tf::Quaternion q):w(q.getW()),x(q.getX()),y(q.getY()),z(q.getZ()){}
        friend std::ostream& operator<< (std::ostream &out, Quaternion &quaternion)
        {
            out << "[w: " << quaternion.w << ", x: " << quaternion.x << ", y: "<< quaternion.y << ", z: "<< quaternion.z << "]" << std::endl;
            out << displayRPY(tf::Quaternion(quaternion.x,quaternion.y,quaternion.z,quaternion.w)) << std::endl;
            return out;
        }
        geometry_msgs::Quaternion to_geometry_quaternion()
        {
            geometry_msgs::Quaternion quaternion_msg;
            quaternion_msg.w = w;
            quaternion_msg.x = x;
            quaternion_msg.y = y;
            quaternion_msg.z = z;
            return quaternion_msg;
        }
    };

    //in degree
    struct PanTiltAngle
    {
        UWB_DATA_TYPE theta1; //in degree
        UWB_DATA_TYPE theta2; //in degree
        PanTiltAngle():theta1(0),theta2(0){}
        PanTiltAngle(UWB_DATA_TYPE theta1, UWB_DATA_TYPE theta2):theta1(theta1), theta2(theta2){}
        friend std::ostream& operator<< (std::ostream &out, PanTiltAngle &panTiltAngle)
        {
            out << "[theta1 : " << panTiltAngle.theta1 << ", theta2: "<< panTiltAngle.theta2 << "]" << std::endl;
            return out;
        }
        tf::Transform get_tf_transform_base_s1()
        {
            tf::Transform base_s1;
            base_s1.setOrigin(tf::Vector3(0,0,h1));
            base_s1.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -theta1/180*M_PI));
            return base_s1;
        }
        tf::Transform get_tf_transform_s1_s2()
        {
            tf::Transform s1_s2;
            s1_s2.setOrigin(tf::Vector3(0,0,h2));
            s1_s2.setRotation(tf::Quaternion(tf::Vector3(0,1,0),-theta2/180*M_PI));
            return s1_s2;
        }
        tf::Transform get_tf_transform_s2_cam()
        {
            tf::Transform s2_cam;
            s2_cam.setOrigin(tf::Vector3(h4,0,h3));
            tf::Quaternion s2_cam_q;
            tf::Matrix3x3 s2_cam_mat(
                 0,0,1,
                -1,0,0,
                0,-1,0
            );// x_cam = -y_s2, y_cam = -z_s2, z_cam = x_s2
            s2_cam_mat.getRotation(s2_cam_q);
            s2_cam.setRotation(s2_cam_q);
            return s2_cam;
        }
        tf::Transform get_tf_transform()
        {
            return get_tf_transform_base_s1() * get_tf_transform_s1_s2() * get_tf_transform_s2_cam();
        }
    };

    //size 7*4=28
    struct Pose
    {
        Vector3 position;
        Quaternion orientation;
        Pose(){}
        Pose(tf::Transform t)
        {
            position = Vector3(t.getOrigin().getX(),t.getOrigin().getY(),t.getOrigin().getZ());
            orientation = Quaternion(t.getRotation());
        }

        Pose(geometry_msgs::Pose p):position(p.position),orientation(p.orientation){}

        Pose(std::vector<double> rvec, std::vector<double> tvec)
        {
            position = Vector3(tvec[0],tvec[1],tvec[2]);
            orientation = Quaternion(cvtToTFQuaternion(rvec));
        }

        Pose(std::vector<double> tvec)
        {
            position = Vector3(tvec[0],tvec[1],tvec[2]);
            orientation = Quaternion(1,0,0,0);
        }

      
        friend std::ostream& operator<< (std::ostream &out, Pose &pose)
        {
            out << "    position: " << pose.position;
            out << "    orientation: " << pose.orientation;
            return out;
        }
        geometry_msgs::Pose to_geometry_pose()
        {
            geometry_msgs::Pose pose_msg;
            pose_msg.position = position.to_geometry_point();
            pose_msg.orientation = orientation.to_geometry_quaternion();
            return pose_msg;
        }
        tf::Transform to_tf_transform()
        {
            return tf::Transform(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
                                tf::Vector3(position.x, position.y, position.z));
        }
    };

    //size 6*4=24
    struct TwistOrAccel
    {
        Vector3 linear;
        Vector3 angular;
        friend std::ostream& operator<< (std::ostream &out, TwistOrAccel &twistOrAccel)
        {
            out << "    linear: " << twistOrAccel.linear;
            out << "    angular: " << twistOrAccel.angular;
            return out;
        }
        geometry_msgs::Twist to_geometry_twist()
        {
            geometry_msgs::Twist twist_msg;
            twist_msg.linear = linear.to_geometry_vector3();
            twist_msg.angular = angular.to_geometry_vector3();
            return twist_msg;
        }
        geometry_msgs::Accel to_geometry_accel()
        {
            geometry_msgs::Accel accel_msg;
            accel_msg.linear = linear.to_geometry_vector3();
            accel_msg.angular = angular.to_geometry_vector3();
            return accel_msg;
        }

    };

    //size 1+28+24=53
    struct ObservedState
    {
        uint8_t id = 255;
        Pose pose;
        // TwistOrAccel twist;
        friend std::ostream& operator<< (std::ostream &out, ObservedState &observedState)
        {
            out << "[\nid: " << int(observedState.id) << std::endl;
            out << "  pose: " << std::endl << observedState.pose;
            // out << "  twist: " << std::endl << observedState.twist;
            out << "]\n";
            return out;
        }
    };

    //size 4+8+8+28+53*MAX_DRONE_NUMBER=101,154,207
    struct ObservationMessage
    {
        uint32_t systemTime = 0;
        uint8_t servoControlMethod = 0;
        PanTiltAngle panTiltAngle;
        PanTiltAngle panTiltVelocity;
        Pose world_to_base;
        ObservedState observedState[MAX_DRONE_NUMBER];
        friend std::ostream& operator<< (std::ostream &out, ObservationMessage &observationMessage)
        {
            out << "=====\n";
            out << "systemTime: " << observationMessage.systemTime << std::endl;
            out << "panTiltAngle: " <<  observationMessage.panTiltAngle;
            out << "panTiltVelocity: " <<  observationMessage.panTiltVelocity;
            out << "{\n";
            for(int i = 0; i < MAX_DRONE_NUMBER; i++)
                out << observationMessage.observedState[i];
            out << "}";
            out << "\n";
            return out;
        }
        tf::Transform get_tf_transform_world(int observedState_index = 0)
        {
            tf::Transform world_to_base_tf(world_to_base.to_tf_transform());
            tf::Transform base_to_cam_tf(panTiltAngle.get_tf_transform());
            tf::Transform cam_to_drone_tf(observedState[observedState_index].pose.to_tf_transform());
            // return world_to_base_tf * base_to_cam_tf * cam_to_drone_tf;
            return base_to_cam_tf * cam_to_drone_tf;
        }
        
        geometry_msgs::Pose get_geometry_pose_world(int observedState_index = 0)
        {
            geometry_msgs::Pose geometry_pose_world;
            tf::Transform tf_transform_world = get_tf_transform_world(observedState_index);
            geometry_pose_world.position.x = tf_transform_world.getOrigin().getX();
            geometry_pose_world.position.y = tf_transform_world.getOrigin().getY();
            geometry_pose_world.position.z = tf_transform_world.getOrigin().getZ();
            geometry_pose_world.orientation.w = tf_transform_world.getRotation().getW();
            geometry_pose_world.orientation.x = tf_transform_world.getRotation().getX();
            geometry_pose_world.orientation.y = tf_transform_world.getRotation().getY();
            geometry_pose_world.orientation.z = tf_transform_world.getRotation().getZ();
            return geometry_pose_world;
        }

    };

    //size 4+28+24+24=80
    struct EstimationMessage
    {
        uint32_t systemTime = 0;
        Pose pose;
        // TwistOrAccel twist;
        // TwistOrAccel accel;
        friend std::ostream& operator<< (std::ostream &out, EstimationMessage &estimationMessage)
        {
            out << "=======\n";
            out << "systemTime: " << estimationMessage.systemTime << std::endl;
            out << "pose: " << std::endl << estimationMessage.pose;
            // out << "twist: " << std::endl << estimationMessage.twist;
            // out << "accel: " << std::endl << estimationMessage.accel;
            out << "\n";
            return out;
        }
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

    template<typename T>
    T convertToMessage(const std::string & s)
    {
        T message;
        memcpy((void*)(&message), (void*)(s.c_str()), sizeof(T));
        return message;
    }


    tf::Transform convertPoseTransform(geometry_msgs::Pose p)
    {
        return tf::Transform(tf::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
                                tf::Vector3(p.position.x, p.position.y, p.position.z));
    }
    geometry_msgs::Pose convertPoseTransform(tf::Transform t)
    {
        geometry_msgs::Pose p;
        p.position.x = t.getOrigin().getX();
        p.position.y = t.getOrigin().getY();
        p.position.z = t.getOrigin().getZ();
        p.orientation.w = t.getRotation().getW();
        p.orientation.x = t.getRotation().getX();
        p.orientation.y = t.getRotation().getY();
        p.orientation.z = t.getRotation().getZ();
        return p;
    }

    PanTiltAngle rotateTowards(tf::Transform base_to_drone_tf)
    {
        tf::Transform drone_to_board_tf;
        drone_to_board_tf.setOrigin(tf::Vector3(BOARD_CENTER_X, BOARD_CENTER_Y, BOARD_CENTER_Z));
        geometry_msgs::Pose drone_pose_base = convertPoseTransform(base_to_drone_tf * drone_to_board_tf);
        PanTiltAngle result;
        result.theta1 = -atan2(drone_pose_base.position.y, drone_pose_base.position.x) /M_PI*180;
        tf::Transform s2_to_drone_tf;
        s2_to_drone_tf = (result.get_tf_transform_base_s1() * result.get_tf_transform_s1_s2()).inverse() 
                                * base_to_drone_tf * drone_to_board_tf;
        geometry_msgs::Pose drone_pose_s2 = convertPoseTransform(s2_to_drone_tf);
        double x = drone_pose_s2.position.x;
        double y = drone_pose_s2.position.z;
        result.theta2 = (acos(h3/sqrt(x*x+y*y)) - atan2(x,y))/M_PI*180;
        return result;
    }

}

#pragma pack(pop)

#endif