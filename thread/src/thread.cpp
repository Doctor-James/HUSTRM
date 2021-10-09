#include "thread.h"
namespace ly
{
    std::shared_ptr<SharedData> thread::share_ = std::make_shared<SharedData>();
    void thread::start(int delay)
    {
        delay_ = delay;
        pthread_create(&thread_, NULL, pthread_fun, (void *)this);
    }
    void *thread::pthread_fun(void *__this)
    {
        auto *_this = (thread *)__this;
        while (true)
        {
            try
            {
                _this->process();
            }
            catch (...)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(_this->delay_));
        }
    }
}
