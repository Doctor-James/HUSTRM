#include "serialPortWriteThread.h"
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
            write_ = new serialPort(config.deviceName);
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
        share_->getSendMsg(temp);
        //起始位
        msg[0] = '!';

        unsigned char *tmp = (unsigned char *)(&temp.shootStatus);
        msg[1] = tmp[0];
        //pitch角
        tmp = (unsigned char *)(&temp.pitch);
        msg[2] = tmp[1];
        msg[3] = tmp[0];
        //yaw角
        tmp = (unsigned char *)(&temp.yaw);
        msg[4] = tmp[3];
        msg[5] = tmp[2];
        msg[6] = tmp[1];
        msg[7] = tmp[0];
        //距离
        msg[8] = temp.distance;
        //终止位
        msg[9] = '#';
        write_->Append_CRC8_Check_Sum(msg, max_receive_len_);
        //        std::cout<<"shoot:"<<(int)tmp[0]<<std::endl;
        //	    std::cout<<"send pitch:"<<temp.pitch<<std::endl;
        //	    std::cout<<"send yaw:"<<temp.yaw<<"\n";
        //std::cout << "send data" << std::endl;
        write_->SP_Write(msg, max_receive_len_);
    }
}
