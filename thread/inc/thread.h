#ifndef __THREAD_H
#define __THREAD_H

#include <thread>
#include "SharedData.h"
#include <unistd.h>
#include <chrono>
namespace ly
{
    // 多线程类，纯虚类
    class thread
    {
    public:
        thread() = default;
        ~thread() = default;
        void start(int delay);
        static void *pthread_fun(void *__this);
        virtual void process() = 0;
        static std::shared_ptr<SharedData> share_;
        time counter_;
        virtual void join() { pthread_join(thread_, NULL); };

    private:
        int delay_;
        pthread_t thread_{};
    };
}
#endif //__THREAD_H
