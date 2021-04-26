//
// Created by ljy on 2021/2/1.
//

#ifndef SRC_STAMPEDQUEUE_H
#define SRC_STAMPEDQUEUE_H
#include <iostream>
#include <deque>
#include <utility>
#include "UWB_message_type.h"
#include <geometry_msgs/Vector3.h>


template<class T>
class StampedQueue
{
public:
    const int queue_too_long_warning_size = 200;
    const int queue_buffer_time_ms = 1000;
    std::deque<T> q;
    void addMessage(T msg)
    {
        q.push_back(msg);
        while(!q.empty() && (ros::Time::now() - q.front().header.stamp).toSec() > queue_buffer_time_ms/1000.0)
        {
            q.pop_front();
        }
    }
    bool getMessage(const ros::Time& timestamp, T& outputMsg)
    {
        if(q.size() > queue_too_long_warning_size)
        {
            ROS_WARN_STREAM_THROTTLE(1,"q.size() = " << q.size() << " > queue_too_long_warning_size "
                                                     << queue_too_long_warning_size);
        }
        if(q.empty())
        {
            ROS_ERROR("queue is empty!");
            return false;
        }
        if(q.size() == 1)
        {
            ROS_WARN_STREAM("q.size() == 1, timestamp LATER than msg.timestamp " << (q.front().header.stamp - timestamp).toSec());
            outputMsg = q.front();
            return true;
        }

        double average_msg_interval = (q.back().header.stamp - q.front().header.stamp).toSec() / (q.size()-1);
        if((q.front().header.stamp - timestamp).toSec() > average_msg_interval)
        {
            ROS_ERROR_STREAM("timestamp is earlier than queue front " << (q.front().header.stamp - timestamp).toSec() << " s.");
            ROS_ERROR_STREAM("q.front().header " << q.front().header.stamp.toSec() << " timestamp_want: "  << timestamp.toSec() << " s.");
            return false;
        }
        if((timestamp - q.back().header.stamp).toSec() > average_msg_interval)
        {
            ROS_ERROR_STREAM("timestamp is later than queue back " << (timestamp - q.back().header.stamp).toSec() << " s.");
            return false;
        }

        int front = 0, rear = q.size() - 1;
        while(front < rear)
        {
            int middle = (front + rear) / 2;
            if(q[middle].header.stamp < timestamp) front = middle + 1;
            else rear = middle;
        }

        if(front > 0)
        {
            if(( q[front].header.stamp - timestamp).toSec()<(timestamp - q[front - 1].header.stamp).toSec())
            {
                outputMsg = q[front];
            }
            else
            {
                outputMsg = q[front - 1];
            }
        }
        else
        {
            outputMsg = q[front];
        }
        ROS_INFO_STREAM_THROTTLE(1, "get StampedMsg from q: T_want - T_get = " << (timestamp - outputMsg.header.stamp).toSec()*1000 << " ms." );
        //ROS_INFO_STREAM("get StampedMsg from q: T_want - T_get = " << (timestamp - outputMsg.header.stamp).toSec()*1000 << " ms." );
        
        return true;
    }
};



#endif //SRC_STAMPEDQUEUE_H
