#ifndef _LOCKFREE_QUEUE_
#define _LOCKFREE_QUEUE_
#include <iostream>
#include <atomic>
namespace ly
{
#define FIXED_NUM 8 //设置为2的整数倍方便处理
    template <typename T>
    class LockFreeQueue
    {
        struct Element
        {
            T data_;
            std::atomic<bool> is_full; //是否有数据
        };

    public:
        LockFreeQueue();
        ~LockFreeQueue();
        bool push(T &input);
        bool pop(T &output);
        std::atomic<short> write_index;
        Element data_queue[FIXED_NUM];
    };
    template <typename T>
    LockFreeQueue<T>::LockFreeQueue()
    {
        write_index = 0;
    }
    template <typename T>
    LockFreeQueue<T>::~LockFreeQueue()
    {
    }
    template <typename T>
    bool LockFreeQueue<T>::push(T &input)
    {
        //由于后面的语句都对write_index_temp有依赖，故此处的内存模型都无所谓
        short write_index_temp = write_index.load(std::memory_order_relaxed);
        data_queue[write_index_temp].data_ = std::move(input);
        //强制前一条语句在这一条之前运行，即必须要修改完data_在将is_full置true
        data_queue[write_index_temp].is_full.store(true, std::memory_order_release);
        write_index_temp++;
        write_index_temp %= 8;
        write_index.store(write_index_temp, memory_order::memory_order_release);
        return true;
    }
    template <typename T>
    bool LockFreeQueue<T>::pop(T &output)
    {
        short read_index = write_index.load(std::memory_order::memory_order_relaxed);
        read_index = (read_index + 7) % 8;
        //强制后面的语句在这一条语句之后运行,即必须要is_full为true才执行后面的代码
        if (data_queue[read_index].is_full.load(std::memory_order_acquire))
        {
            output = std::move(data_queue[read_index].data_);
            data_queue[read_index].is_full.store(false, std::memory_order_release);
            return true;
        }
        return false;
    }
}

#endif