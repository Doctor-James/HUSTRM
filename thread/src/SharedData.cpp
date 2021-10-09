#include "SharedData.h"
namespace ly
{
    bool SharedData::is_armor_getable = false;
    bool SharedData::is_lightBar_getable = false;
    bool SharedData::is_serial_receive_getable = false;
    bool SharedData::is_serial_send_getable = false;
    bool SharedData::is_pic_getable = false;
    SharedData::SharedData()
    {
    }
    SharedData::~SharedData()
    {
    }
    void SharedData::setPic(Mat &input_frame) //生产图片
    {
        // {
        //     std::unique_lock<std::mutex> mutex_(mutex_pic_);
        //     is_pic_getable = false;
        //     frame_.mat = input_frame.mat.clone();
        //     frame_.time = clock();
        // }
        // is_pic_getable = true;
        // condition_pic_.notify_one();
        // pic_queue.push(input_frame);
        my_pic_queue.push(input_frame);
    }
    void SharedData::getPic(Mat &output_frame)
    {
        // std::unique_lock<std::mutex> mutex_(mutex_pic_);
        // condition_pic_.wait(mutex_, []()
        //                     { return is_pic_getable; });
        // output_frame.mat = frame_.mat.clone();
        // output_frame.time = frame_.time;
        // is_pic_getable = false;
        // while (!pic_queue.pop(output_frame))
        // {
        //     std::this_thread::yield();
        // };
        while (!my_pic_queue.pop(output_frame))
        {
            std::this_thread::yield();
        }
    }
    void SharedData::setLightbar(std::priority_queue<lightBarNode> lightBar_)
    {
        // {
        //     std::unique_lock<std::mutex> mutex_(mutex_lightBar_);
        //     is_lightBar_getable = false;
        //     lightBarsQue_ = lightBar_;
        // }
        // is_lightBar_getable = true;
        // condition_lightBar_.notify_one();
        my_lightBar_queue.push(lightBar_);
    }
    void SharedData::getLightbar(std::priority_queue<lightBarNode> &lightBar_)
    {
        // {
        //     std::unique_lock<std::mutex> mutex_(mutex_lightBar_);
        //     condition_lightBar_.wait(mutex_, []()
        //                              { return is_lightBar_getable; });
        //     lightBar_ = lightBarsQue_;
        // }
        // is_lightBar_getable = false;
        while (!my_lightBar_queue.pop(lightBar_))
        {
            std::this_thread::yield();
        }
    }
    void SharedData::setArmor(std::priority_queue<armorNode> armor_)
    {
        {
            std::unique_lock<std::mutex> mutex_(mutex_armor_);
            is_armor_getable = false;
            armorQue_ = armor_;
        }
        is_armor_getable = true;
        condition_armor_.notify_one();
    }
    void SharedData::getArmor(std::priority_queue<armorNode> &armor_)
    {
        {
            std::unique_lock<std::mutex> mutex_(mutex_armor_);
            condition_armor_.wait_for(mutex_, std::chrono::microseconds(10), []()
                                      { return is_armor_getable; });
            armor_ = armorQue_;
        }
        is_armor_getable = false;
    }
    void SharedData::setRecieveMsg(receiveData receive_)
    {
        {
            std::unique_lock<std::mutex> mutex_(mutex_serial_receive_);
            is_serial_receive_getable = false;
            if (receiveData_.size() > 149)
            {
                receiveData_.erase(receiveData_.begin());
            }
            receiveData_.emplace_back(receive_);
        }
        is_serial_receive_getable = true;
        condition_serial_receive_.notify_one();
    }
    void SharedData::getRecieveMsg(std::vector<ly::receiveData> &receive_)
    {
        std::vector<ly::receiveData> data;
        {
            std::unique_lock<std::mutex> mutex_(mutex_serial_receive_);
            condition_serial_receive_.wait(mutex_, []()
                                           { return is_serial_receive_getable; });
            receive_ = receiveData_;
        }
        is_serial_receive_getable = false;
    }
    void SharedData::setSendMsg(sendData send_)
    {
        {
            std::unique_lock<std::mutex> mutex_(mutex_serial_send_);
            is_serial_send_getable = false;
            sendData_ = send_;
        }
        is_serial_send_getable = true;
        condition_serial_send_.notify_one();
    }
    void SharedData::getSendMsg(sendData &send_)
    {
        {
            std::unique_lock<std::mutex> mutex_(mutex_serial_send_);
            condition_serial_send_.wait(mutex_, []()
                                        { return is_serial_send_getable; });
            send_ = sendData_;
        }
        is_serial_send_getable = false;
    }
}
