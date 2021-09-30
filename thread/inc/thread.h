/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-22 10:53:34
 */
/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-19 21:28:10
 */
#ifndef __THREAD_H
#define __THREAD_H

#include <mutex>
#include <thread>
#include <condition_variable>

namespace ly
{
    static std::mutex mutex_pic_;
    static std::condition_variable condition_pic_;
    static bool is_pic_getable = true;
    // 多线程类，纯虚类
    class thread
    {
    public:
        //定义锁，条件变量，条件判断量
        static std::mutex mutex_serial_receive_;
        static std::mutex mutex_serial_send_;
        //static std::mutex mutex_pic2_;
        static std::mutex mutex_armor_;
        static std::mutex mutex_lightBar_;
        static std::mutex mutex_ROI_;
        //static std::condition_variable condition_pic2_;
        static std::condition_variable condition_armor_;
        static std::condition_variable condition_lightBar_;
        static std::condition_variable condition_serial_receive_;
        static std::condition_variable condition_serial_send_;
        static std::condition_variable condition_ROI_;
        //static bool is_pic2_getable;
        static bool is_armor_getable;
        static bool is_lightBar_getable;
        static bool is_serial_receive_getable;
        static bool is_serial_send_getable;
        static bool is_ROI_getable;

        thread() = default;
        ~thread() = default;
        void start(int delay);
        static void *pthread_fun(void *__this);
        int delay_;
        virtual void process() = 0;
        pthread_t thread_{};
    };
}
#endif //__THREAD_H
