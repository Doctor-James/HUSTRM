#ifndef __SERIAL_READ_PORT_THREAD_H
#define __SERIAL_READ_PORT_THREAD_H

#include "serialPort.h"
#include "tools.h"
#include "thread.h"
#include <iostream>
#include <utility>
#include "tool_log.h"
namespace ly
{
    // 串口设备的接收接口类
    class serialPortReadThread : public thread
    {
    public:
        serialPortReadThread() = default;
        explicit serialPortReadThread(const serialPort_dev &config);
        ~serialPortReadThread();

    private:
        serialPort *read_;
        unsigned char temptemp[2 * 10];
        unsigned char temp[10];
        int max_receive_len_ = 10;
        unsigned char read_data_[10];
        receiveData stm32;
        void process() override;
        void readData();
    };
}
#endif //__SERIAL_PORT_THREAD_H
