#ifndef __TOOL_H
#define __TOOL_H

#include "tool_time.h"
#include "tool_config.h"
#include "tool_Drawcurve.h"
#include <opencv2/opencv.hpp>

namespace ly{
    // 接收数据格式
    class receiveData{
    public:
        unsigned char flag;
        int yaw;
        short pitch;
        int MCU_Time;   //接收到的单片机的时间戳（FreeRTOS）
        struct timeval TX2_BeginTime; //收到的第一帧单片机时间戳作为辅瞄代码起始时间戳
        clock_t TX2_Time; // 当前时间戳与辅瞄代码起始时间戳的差值（代码运行时间）
        int delta_t;   //TX2时间轴与MCU时间轴的偏移量，用来动态时间同步
    };

    // 发送数据格式
    class sendData{
    public:
        unsigned char shootStatus;
        int yaw;
        short pitch;
        unsigned char distance;
    };

    typedef struct{
        cv::Mat mat;
        receiveData receiveData_;
        struct timeval timeval; //获取照片读取的时间戳
        clock_t time;
    } Mat;
}

#endif //__TOOL_H

