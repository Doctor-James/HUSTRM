/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-22 10:52:41
 */
#include "thread.h"
#include <unistd.h>
#include <chrono>
namespace ly
{
    //初始化锁和条件变量和条件判断量
    std::mutex thread::mutex_serial_receive_;
    std::mutex thread::mutex_serial_send_;
    std::mutex thread::mutex_armor_;
    std::mutex thread::mutex_lightBar_;
    std::mutex thread::mutex_ROI_;
    std::condition_variable thread::condition_armor_;
    std::condition_variable thread::condition_lightBar_;
    std::condition_variable thread::condition_serial_receive_;
    std::condition_variable thread::condition_serial_send_;
    std::condition_variable thread::condition_ROI_;
    bool thread::is_armor_getable = false;
    bool thread::is_lightBar_getable = false;
    bool thread::is_serial_receive_getable = false;
    bool thread::is_serial_send_getable = false;
    bool thread::is_ROI_getable = false;

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
