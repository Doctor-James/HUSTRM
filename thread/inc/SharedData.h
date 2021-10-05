#ifndef _SHARED_DATA_H
#define _SHARED_DATA_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include "tools.h"
#include <mutex>
#include <condition_variable>
#include "armor.h"
#include "lightBar.h"
#include <thread>
#define PHOTO_HEIGHT 1024
#define PHOTO_WIDTH 1280
namespace ly
{
    class SharedData
    {
    private:
        std::mutex mutex_serial_receive_;
        std::mutex mutex_serial_send_;
        std::mutex mutex_pic_;
        std::mutex mutex_armor_;
        std::mutex mutex_lightBar_;

        std::condition_variable condition_armor_;
        std::condition_variable condition_lightBar_;
        std::condition_variable condition_serial_receive_;
        std::condition_variable condition_serial_send_;
        std::condition_variable condition_pic_;

        static bool is_armor_getable;
        static bool is_lightBar_getable;
        static bool is_serial_receive_getable;
        static bool is_serial_send_getable;
        static bool is_pic_getable;

        Mat frame_;
        std::priority_queue<lightBarNode> lightBarsQue_;
        std::priority_queue<armorNode> armorQue_;
        std::vector<ly::receiveData> receiveData_;
        sendData sendData_;

    public:
        SharedData();

        ~SharedData();

        void setPic(Mat &input_frame);

        void getPic(Mat &output_frame);

        void setLightbar(std::priority_queue<lightBarNode> lightBar_);

        void getLightbar(std::priority_queue<lightBarNode> &lightBar_);

        void setArmor(std::priority_queue<armorNode> armor_);

        void getArmor(std::priority_queue<armorNode> &armor_);

        void setRecieveMsg(receiveData receive_);

        void getRecieveMsg(std::vector<ly::receiveData> &receive_);

        void setSendMsg(sendData send_);

        void getSendMsg(sendData &send_);
    };

}

#endif