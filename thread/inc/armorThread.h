/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-22 10:41:33
 */

#ifndef __ARMOR_THREAD_H
#define __ARMOR_THREAD_H
#include "solvePNP.h"
#include "armor.h"
#include "thread.h"
#include "lightBarThread.h"
#include "serialPortReadThread.h"
#include "serialPortWriteThread.h"
#include <mutex>
#include <thread>

#define PREDICT 0
#define VISUALIZE 0

#define constraint(a, min, max) \
    fmin(fmax(a, min), max) //限制a在min与max之间

namespace ly
{
    class armorThread : public armor, public thread
    {
    public:
        armorThread() = default;
        explicit armorThread(armor_param config, armor_param large, cam_param cam_config, lightBarThread *lightBar, serialPortReadThread *serialPortRead, serialPortWriteThread *serialPortWrite, cameraThread *camera);
        ~armorThread();

    private:
        void process() override;
        bool verifyArmor(cv::Mat pic, std::priority_queue<armorNode> armor);
        cv::Mat getArmorRoi(armorNode);
        void updateArmor();
        lightBarThread *lightBar_;
        serialPortReadThread *serialPortRead_;
        serialPortWriteThread *serialPortWrite_;
        solvePNP *pnp_;
        cameraThread *camera_;
        score *score_;
        bool debug_ = true;     //debug模式
        bool update_ = false;   //是否已经pnp解算
        receiveData shootType_; //收到的信息

        armorNode best_armor_; //最优装甲板信息
        time counter_;         //计时器

        std::string path = "../data/pic_blue/tmp/img_"; //图片保存地址

        bool isLoad = false;        //是否加载了模型
        cv::Ptr<cv::ml::SVM> model; //svm大小装甲板分类

        cv::Size roiSize = cv::Size(32, 32); //ROI,32x32
    };
}

#endif //__ARMOR_THREAD_H
