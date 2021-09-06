/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-23 10:10:06
 */
/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-19 09:58:54
 */
#include "serialPortWriteThread.h"
#include <iostream>
#include <utility>
#include "tool_log.h"

namespace ly
{
    /**
     * @brief 构造函数
     * @param portName 串口名称
     */
    serialPortWriteThread::serialPortWriteThread(const serialPort_dev &config)
    {
        if (config.enable)
        {
            init(config.deviceName);
            start(10000);
        }
    }
    serialPortWriteThread::~serialPortWriteThread()
    {
    }
    /**
     * @brief 串口线程函数
     */
    void serialPortWriteThread::process()
    {
        writeData();
    }
    /**
     * @brief 写数据
     */
    void serialPortWriteThread::writeData()
    {
        unsigned char msg[max_receive_len_];
        ly::sendData temp;
        // {
        //     std::unique_lock<std::mutex> mutex_(mutex_serial_send_);
        //     condition_serial_send_.wait(mutex_, []()
        //                                 { return is_serial_send_getable; });
        //     temp = send_;
        // }
        temp.distance = 1;
        temp.shootStatus = 0;
        temp.pitch = 100;
        temp.yaw = 10;
        //@TODO 添加CRC
        msg[0] = '!';
        unsigned char *tmp = (unsigned char *)(&temp.shootStatus);
        msg[1] = tmp[0];
        //        std::cout<<"shoot:"<<(int)tmp[0]<<std::endl;
        tmp = (unsigned char *)(&temp.pitch);
        //	    std::cout<<"send pitch:"<<temp.pitch<<std::endl;
        msg[2] = tmp[1];
        msg[3] = tmp[0];

        tmp = (unsigned char *)(&temp.yaw);
        //	    std::cout<<"send yaw:"<<temp.yaw<<"\n";
        msg[4] = tmp[3];
        msg[5] = tmp[2];
        msg[6] = tmp[1];
        msg[7] = tmp[0];

        msg[8] = temp.distance;

        msg[9] = '#';
        Append_CRC8_Check_Sum(msg, max_receive_len_);
        SP_Write(msg, max_receive_len_);
    }
    /**
     * @brief 设置需要发送的数据
     * @param data 发送数据
     */
    void serialPortWriteThread::setSendMsg(sendData data)
    {
        {
            std::unique_lock<std::mutex> mutex_(mutex_serial_send_);
            is_serial_send_getable = false;
            send_ = data;
        }
        is_serial_send_getable = true;
        condition_serial_send_.notify_one();
    }
}
