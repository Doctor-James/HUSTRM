/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-23 10:26:33
 */
#include "serialPortReadThread.h"
#include <iostream>
#include <utility>
#include "tool_log.h"
namespace ly
{
int last_time_;
    std::vector<receiveData> receive_;
    /**
     * @brief 构造函数
     * @param portName 串口名称
     */
    serialPortReadThread::serialPortReadThread(const serialPort_dev &config)
    {
        if (config.enable)
        {
            init(config.deviceName);
            start(1000);
        }
        stm32.delta_t = 0;
        receive_.resize(50);
    }
    serialPortReadThread::~serialPortReadThread()
    {
        //delete read_data_;
    }
    /**
     * @brief 串口线程函数
     */
    void serialPortReadThread::process()
    {
        readData();
    }
    /**
     * @brief 读数据
     */
    void serialPortReadThread::readData()
    {
        short flag, counter, Gyro_Pitch, Gyro_Yaw;
        SP_Read(read_data_, max_receive_len_);
        //@TODO 读取数据
        memcpy(temptemp + 14, read_data_, 14);

        for (int start_bit = 0; start_bit < 14; start_bit++)
        {
            if (temptemp[start_bit] == '!')
            {
                for (int j = 0; j < 14; j++)
                {
                    temp[j] = temptemp[start_bit + j];
                }
                //@TODO CRC校验
                if (Verify_CRC8_Check_Sum(temp, 14)) //CRC校验
                {
                    stm32.flag = temp[1];
                    stm32.pitch = (temp[2] << 8) | temp[3];
                    stm32.yaw = (((int)temp[4]) << 24) | (((int)temp[5]) << 16) | (((int)temp[6]) << 8) | (temp[7]);
                    stm32.MCU_Time = (((int)temp[8]) << 24) | (((int)temp[9]) << 16) | (((int)temp[10]) << 8) | (temp[11]); //ms
                    //时间戳初始化
                    if(!BeginTime_Flag)
                    {
                        MCU_BeginTime = stm32.MCU_Time;
                        gettimeofday(&Tx2_BeginTime_, NULL);
                        BeginTime_Flag = true;
                    }

                    struct timeval time_;
                    gettimeofday(&time_, NULL);
                    //std::cout<<"time_.tv_sec:"<<time_.tv_sec<<"time_.tv_usec: "<<time_.tv_usec<<std::endl;
                    int Data_Time = (double) (1000000 * (time_.tv_sec-Tx2_BeginTime_.tv_sec) + (time_.tv_usec-Tx2_BeginTime_.tv_usec)) / 1000; //ms
		//stm32.delta_t = (Data_Time) - (stm32.MCU_Time - MCU_BeginTime); 
                    //stm32.TX2_Time = Data_Time-stm32.delta_t;
                stm32.TX2_Time=Data_Time;//(stm32.MCU_Time - MCU_BeginTime);    

		stm32.TX2_BeginTime = Tx2_BeginTime_;
		    int now_time =stm32.MCU_Time - MCU_BeginTime;
	if(last_time_!= now_time)
{
last_time_ = now_time;
//stm32.delta_t = (Data_Time) - (stm32.MCU_Time - MCU_BeginTime);
//std::cout<<stm32.delta_t<<std::endl;
//std::cout<<"MCU_time: "<<stm32.MCU_Time<<"TX2_Time: "<<Data_Time<<std::endl;

std::cout<<"MUC_Time: "<<stm32.TX2_Time<<std::endl;
} 
//std::cout<<"(Data_Time): "<<(Data_Time)<<"stm32.TX2_Time: "<<(stm32.MCU_Time - MCU_BeginTime)<<std::endl;  
  
                //时间同步(时间差大于6ms时进行)
//                    if(abs((stm32.TX2_Time) - (stm32.MCU_Time - MCU_BeginTime)) >4 )
                    {
                       // outrange++;
                       // if(outrange_times > 1)
                        {
//                            stm32.delta_t += (stm32.TX2_Time) - (stm32.MCU_Time - MCU_BeginTime);
                         //   outrange_times = 0;
                        }
                    }


                    //std::cout<<"stm32.MCU_Time:"<<stm32.MCU_Time<<"MCU_BeginTime: "<<MCU_BeginTime<<std::endl;
                    //std::cout<<"stm32.Data_Time:"<<stm32.TX2_Time<<"MUCtime: "<<(stm32.MCU_Time - MCU_BeginTime)<<"delta_t: "<<stm32.delta_t<<std::endl;
                    //std::cout<<(stm32.MCU_Time - MCU_BeginTime)-stm32.Data_Time<<std::endl;
                    //std::cout<<"-----------stm32.pitch:"<<stm32.pitch<<std::endl;
                    //std::cout<<"stm32.yaw:"<<stm32.yaw<<std::endl;
                }
                break;
            }
        }

        memcpy(temptemp, temptemp + 14, 14);
        {
            std::unique_lock<std::mutex> mutex_(mutex_serial_receive_);
            is_serial_receive_getable = false;
            if (receive_.size() > 49)
            {
                receive_.erase(receive_.begin());
            }
            receive_.emplace_back(stm32);
        }
        is_serial_receive_getable = true;
        condition_serial_receive_.notify_one();
    }
    /**
     * @brief 获取收到的数据
     * @return 当前数据
     */
    std::vector<receiveData> serialPortReadThread::getReceiveMsg()
    {
        std::vector<ly::receiveData> data;
        {
            std::unique_lock<std::mutex> mutex_(mutex_serial_receive_);
            condition_serial_receive_.wait(mutex_, []()
                                           { return is_serial_receive_getable; });
            data = receive_;
        }
        return data;
    }
}
