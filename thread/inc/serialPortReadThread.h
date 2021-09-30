#ifndef __SERIAL_READ_PORT_THREAD_H
#define __SERIAL_READ_PORT_THREAD_H

#include "serialPort.h"
#include <mutex>
#include <thread>
#include "tools.h"
#include "thread.h"

namespace ly
{
    // 串口设备的接收接口类
    class serialPortReadThread:public serialPort,public thread
    {
    public:
        serialPortReadThread() = default;
        explicit serialPortReadThread(const serialPort_dev& config);
        ~serialPortReadThread();
        std::vector<receiveData> getReceiveMsg();
    private:
        unsigned char temptemp[2*14];
        unsigned char temp[14];
        int max_receive_len_ = 14;
        unsigned char read_data_[14];
        receiveData stm32;
        void process() override;
        void readData();


        bool BeginTime_Flag = false;
        int MCU_BeginTime;
        struct timeval Tx2_BeginTime_;
        int outrange_times = 0;
        time counter_;
	int last_time;
    };
}
#endif //__SERIAL_PORT_THREAD_H
