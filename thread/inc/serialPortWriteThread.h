#ifndef __SERIAL_WRITE_PORT_THREAD_H
#define __SERIAL_WRITE_PORT_THREAD_H

#include "serialPort.h"
#include "tools.h"
#include "thread.h"
#include <iostream>
#include <utility>
#include "tool_log.h"
namespace ly
{
    // 串口设备的发送接口类
    class serialPortWriteThread : public thread
    {
    public:
        serialPortWriteThread() = default;
        explicit serialPortWriteThread(const serialPort_dev &config);
        ~serialPortWriteThread();

    private:
        serialPort *write_;
        int max_receive_len_ = 10;
        void process() override;
        void writeData();
    };
}
#endif //__SERIAL_PORT_THREAD_H
