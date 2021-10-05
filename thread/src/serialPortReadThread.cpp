#include "serialPortReadThread.h"
namespace ly
{
    /**
     * @brief 构造函数
     * @param portName 串口名称
     */
    serialPortReadThread::serialPortReadThread(const serialPort_dev &config)
    {
        if (config.enable)
        {
            read_ = new serialPort(config.deviceName);
            start(100);
        }
    }
    serialPortReadThread::~serialPortReadThread()
    {
    }
    /**
     * @brief 串口线程函数
     */
    void serialPortReadThread::process()
    {
        readData();
        share_->setRecieveMsg(stm32);
    }
    /**
     * @brief 读数据
     */
    void serialPortReadThread::readData()
    {
        short flag, counter, Gyro_Pitch, Gyro_Yaw;
        read_->SP_Read(read_data_, max_receive_len_);
        std::cout << "read data" << std::endl;
        memcpy(temptemp + 10, read_data_, 10);

        for (int start_bit = 0; start_bit < 10; start_bit++)
        {
            if (temptemp[start_bit] == '!')
            {
                for (int j = 0; j < 10; j++)
                {
                    temp[j] = temptemp[start_bit + j];
                }
                if (read_->Verify_CRC8_Check_Sum(temp, 10)) //CRC校验
                {
                    stm32.flag = temp[1];
                    stm32.pitch = (temp[2] << 8) | temp[3];
                    stm32.yaw = (((int)temp[4]) << 24) | (((int)temp[5]) << 16) | (((int)temp[6]) << 8) | (temp[7]);
                    struct timeval time_;
                    gettimeofday(&time_, NULL);
                    stm32.Data_Time = time_.tv_sec;
                    //std::cout<<"-----------stm32.pitch:"<<stm32.pitch<<std::endl;
                    //std::cout<<"stm32.yaw:"<<stm32.yaw<<std::endl;
                }
                break;
            }
        }

        memcpy(temptemp, temptemp + 10, 10);
    }
}
