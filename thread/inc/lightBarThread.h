/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-22 10:44:11
 */
#ifndef __LIGHT_BAR_THREAD_H
#define __LIGHT_BAR_THREAD_H
#include "lightBar.h"
#include "thread.h"
#include "camera.h"
#include "preProcessThread.h"
#include "armor.h"
#include "cameraThread.h"
#include "buff.h"
namespace ly
{
    class lightBarThread : public lightBar, public thread
    {
    public:
        explicit lightBarThread(cam_param cam_config, lightBar_param config, cameraThread *camera, preProcessThread *preProcess, serialPortReadThread *serialPortRead, serialPortWriteThread *serialPortWrite);
        lightBarThread() = default;
        Mat pic_;
        /*!
         * @name getLightBarQue
         * @brief 如果需要更新且当前lightbar队列不为空，就更新；否则就返回空队列
         * @param none
         * @return lightbar队列
         */
        std::priority_queue<lightBarNode> getLightBarQue() override;
        /*!
         * @name setArmorQue
         * @brief 将armor储存到armor_中
         * @param armor
         * @return none
         */
        void setArmorQue(std::priority_queue<armorNode> armor);
        std::priority_queue<armorNode> getArmorQue();
        cv::Mat show_;

    private:
        cameraThread *camera_ = nullptr;
        preProcessThread *preProcess_ = nullptr;
        serialPortWriteThread *serialPortWrite_ = nullptr;
        serialPortReadThread *serialPortRead_ = nullptr;
        std::priority_queue<armorNode> armor_;
        void getGammaCorrection(cv::Mat &src, cv::Mat &dst);
        std::priority_queue<lightBarNode> ROI(std::priority_queue<lightBarNode> lightBar);
        void process() override;
        void getPreQue();
        void updatePreQue();
        void debug_show();
        void update_lightBar(std::priority_queue<lightBarNode> node);
        void getGammaTable();
        float gamma;
        time counter_;                             //计时器
        bool debug_show_ROI = false;               //可视化ROI
        bool debug_show_light = true;             //可视化lightbar
        bool debug_show_armor = true;              //可视化armor
        std::priority_queue<lightBarNode> preQue_; //预处理的结果队列
        bool update_ = false;                      //检查更新标志
        BUFF_Detector *buff_Tool;
        std::vector<receiveData> receive_Data; //接收到的信息
        int colour;
        cv::Mat table;
        struct timeval BeginTime, end;
        float width_ceo = 3;    //全图检测时roi宽度的放大倍数
        float length_ceo = 1.5; //全图检测时roi长度的放大倍数
    };
}
#endif //__LIGHT_BAR_THREAD_H
