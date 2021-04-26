//
// Created by ljy on 2021/2/2.
//

#ifndef SRC_TIMECONVERTER_H
#define SRC_TIMECONVERTER_H
#include <ros/ros.h>

class TimeConverter {
public:
    uint32_t systemTime = 0;
    double timeBias = -UINT32_MAX/1000.0;
    ros::Time convert_system_time(uint32_t sys_time)
    {
        return ros::Time(sys_time/1000.0 + timeBias);
    }

    uint32_t convert_system_time(ros::Time rostime)
    {
        return uint32_t((rostime.toSec()- timeBias)*1000);
    }
    void updateBias(uint32_t latestSystemTime)
    {
        systemTime = latestSystemTime;
        timeBias = std::max(timeBias, ros::Time::now().toSec() - systemTime/1000.0);
    }
    bool isReady()
    {
        return timeBias != -UINT32_MAX/1000.0;
    }
};


#endif //SRC_TIMECONVERTER_H
