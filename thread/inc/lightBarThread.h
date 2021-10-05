#ifndef __LIGHT_BAR_THREAD_H
#define __LIGHT_BAR_THREAD_H
#include "lightBar.h"
#include "thread.h"
#include "camera.h"
#include "tools.h"
namespace ly
{
    class lightBarThread : public thread
    {
    public:
        explicit lightBarThread(cam_param cam_config, lightBar_param config);
        lightBarThread() = default;
        ~lightBarThread();

    private:
        std::priority_queue<armorNode> armor_;
        Mat pic_;
        std::priority_queue<lightBarNode> ROI(std::priority_queue<lightBarNode> lightBar);
        lightBar *lightBar_;
        void process() override;
        void debug_show();
        std::priority_queue<lightBarNode> preQue_; //预处理的结果队列
        std::vector<receiveData> receive_Data;     //接收到的信息
    };
}
#endif //__LIGHT_BAR_THREAD_H
