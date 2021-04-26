//
// Created by ljy on 2021/2/1.
//

#ifndef SRC_UWBMESSAGEQUEUE_H
#define SRC_UWBMESSAGEQUEUE_H
#include <ros/ros.h>
#include <deque>

template<class T>
class UWBMessageQueue
{
public:
    const int queue_too_long_warning_size = 200;
    const int queue_buffer_time_ms = 1000;
    std::deque<T> q;
    void addMessage(T msg)
    {
        q.push_back(msg);
        while(!q.empty() && q.back().systemTime - q.front().systemTime > queue_buffer_time_ms)
        {
            q.pop_front();
        }
    }
    bool getMessage(const uint32_t system_time, T& outputMsg)
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
            ROS_WARN_STREAM("q.size() == 1, system_time LATER than msg.system_time " << q.front().systemTime - double(system_time) << " ms.");
            outputMsg = q.front();
            return true;
        }
        double average_msg_interval = (q.back().systemTime - q.front().systemTime)/ (q.size()-1);
        if((q.front().systemTime - double(system_time))> average_msg_interval)
        {
            ROS_ERROR_STREAM("system_time is earlier than queue front " << q.front().systemTime - double(system_time) << " ms.");
            return false;
        }
        if((double(system_time) - q.back().systemTime)> average_msg_interval)
        {
            ROS_WARN_STREAM("system_time is later than queue back " << double(system_time) - q.back().systemTime << " ms.");
            outputMsg = q.front();
            return true;
        }

        int front = 0, rear = q.size() - 1;
        while(front < rear)
        {
            int middle = (front + rear) / 2;
            if(q[middle].systemTime < system_time) front = middle + 1;
            else rear = middle;
        }
        if(front > 0)
        {
            if(q[front].systemTime - double(system_time) < double(system_time) - q[front - 1].systemTime)
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

        double min_interval = double(system_time) - q[front].systemTime;
        if(std::abs(min_interval) > average_msg_interval)
            ROS_WARN_STREAM("min_interval > average_msg_interval min_interval =" << min_interval << " ms.");
        ROS_INFO_STREAM("get UWB from q: T_want - T_get = " << min_interval <<" ms." );
        return true;
    }
};


#endif //SRC_UWBMESSAGEQUEUE_H
