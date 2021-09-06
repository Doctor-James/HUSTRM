/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-22 10:49:15
 */
#include <tools.h>
#include "preProcessThread.h"
#include "lightBarThread.h"
#define _DEBUG_

namespace ly
{

    lightBarThread::lightBarThread(cam_param cam_config, lightBar_param config, cameraThread *camera, preProcessThread *preProcess, serialPortReadThread *serialPortRead, serialPortWriteThread *serialPortWrite) : lightBar(config), camera_(camera), preProcess_(preProcess), serialPortRead_(serialPortRead), serialPortWrite_(serialPortWrite)
    {
        gamma = config.gamma;
        colour = 0;//config.color;
        init(std::move(config));
        getGammaTable();
        start(0);
    }
    void lightBarThread::process()
    {
        //时间同步
        pic_ = camera_->getFrame();
        if(pic_.mat.empty())
        {return;}
//        if(serialPortRead_->getReceiveMsg().empty())
//        {return;}
//
//        receive_Data = serialPortRead_->getReceiveMsg();
//        time_t end = receive_Data.at(149).Data_Time;
//
//        double t_ms = double(end-pic_.time);
//        t_ms = fmax(0,fmin(t_ms,149));
//        pic_.receiveData_ = receive_Data.at(149-(int)t_ms);

        colour = 0;
//            cv::Mat temp = pic_.mat.clone();
//            float fGamma = 1/2.2;
//            getGammaCorrection(temp,pic_.mat, fGamma);
        if (pic_.mat.empty())
            return;
        getPreQue();                            //获取预处理得到的前一帧的roi
        detect(pic_, preQue_,colour);        //在roi中检测lightBar
        updatePreQue();                         //用检测到的lightBar更新roi
        mutex_lightBar_.lock();
        copy_lightBarsQue_ = lightBarsQue_;     //更新lightBar
        mutex_lightBar_.unlock();
        update_ = true;
#ifdef _DEBUG_
        debug_show();
#endif
    }
    void lightBarThread::getPreQue() //通过预处理线程获取ROI
    {
        if (preProcess_->checkUpdate())
        {
            preQue_ = preProcess_->getLightBarQue(); //这里实际上返回的是roi
        }
    }
    void lightBarThread::updatePreQue() //更新ROI区域
    {
        preQue_ = ROI(lightBarsQue_); //这里实际上返回的是roi
    }
    void lightBarThread::debug_show()
    {
        // lightBar_time.countEnd();
        // std::cout << 1 / lightBar_time.getTimeMs() * 1000 << std::endl;
        // lightBar_time.countBegin();
        if (debug_show_light || debug_show_armor || debug_show_ROI)
        {
            show_ = pic_.mat.clone();
            std::priority_queue<ly::armorNode> armor_copy = getArmorQue();
            if (debug_show_light) //显示LightBar
            {
                std::priority_queue<lightBarNode> show_lightBar = lightBarsQue_;
                int times = show_lightBar.size() > 5 ? 5 : (int)show_lightBar.size();
                for (int i = 0; i < times; ++i)
                {
                    if (show_lightBar.empty())
                        continue;
                    cv::Point2f rect_points[4];
                    for (int j = 0; j < 4; j++)
                    {
                        line(show_, show_lightBar.top().point[j], show_lightBar.top().point[(j + 1) % 4], 255, 2, 8);
                    }
                    std::stringstream ss;
                    ss << show_lightBar.top().angle;
                    std::string num = ss.str();
                    cv::putText(show_, num, show_lightBar.top().point[0], cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 255, 0));
                    show_lightBar.pop();
                }
            }
            if (!armor_copy.empty() && debug_show_armor) //显示armor
            {
                std::stringstream ss;
                ss << armor_copy.top().lightBarRatio;
                std::string num = ss.str();
                cv::putText(show_, num, armor_copy.top().center, cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(show_, "0", armor_copy.top().corner[0], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(show_, "1", armor_copy.top().corner[1], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(show_, "2", armor_copy.top().corner[2], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(show_, "3", armor_copy.top().corner[3], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);

                for (int j = 0; j < 4; j++)
                {
                    line(show_, armor_copy.top().corner[j], armor_copy.top().corner[(j + 1) % 4], 255, 2, 8);
                }
            }
            if (debug_show_ROI) //显示ROI
            {
                std::priority_queue<lightBarNode> show_lightBar = preQue_;

                int times = show_lightBar.size() > 5 ? 5 : (int)show_lightBar.size();
                for (int i = 0; i < times; ++i)
                {
                    if (show_lightBar.empty())
                        continue;
                    cv::Point2f rect_points[4];
                    for (int j = 0; j < 4; j++)
                    {
                        line(show_, show_lightBar.top().point[j], show_lightBar.top().point[(j + 1) % 4], 255, 2, 8);
                    }
                    std::stringstream ss;
                    ss << i; //lightBar.top().angle;
                    std::string num = ss.str();
                    cv::putText(show_, num, show_lightBar.top().point[0], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                    show_lightBar.pop();
                }
            }
            cv::namedWindow("lightBar debug", 0);
            cv::imshow("lightBar debug", show_);
            cv::waitKey(30);
        }
    }
    std::priority_queue<lightBarNode> lightBarThread::getLightBarQue() //外部获取灯条接口
    {
//        std::priority_queue<lightBarNode> temp;
//        {
//            std::unique_lock<std::mutex> mutex_(mutex_lightBar_);
//            condition_lightBar_.wait(mutex_, []()
//                                     { return is_lightBar_getable; });
//            temp = copy_lightBarsQue_;
//        }
//        is_lightBar_getable = false;
//        return temp;
        if(update_ && !copy_lightBarsQue_.empty())
        {
            update_ = false;
            mutex_lightBar_.lock();
            auto temp = copy_lightBarsQue_;
            mutex_lightBar_.unlock();
            return temp;
        }
        return std::priority_queue<lightBarNode>();
    }
    void lightBarThread::update_lightBar(std::priority_queue<lightBarNode> node) //生产者
    {
        {
            std::unique_lock<std::mutex> mutex_(mutex_lightBar_);
            is_lightBar_getable = false;
            copy_lightBarsQue_ = node;
        }
        is_lightBar_getable = true;
        condition_lightBar_.notify_one();
    }
    void lightBarThread::setArmorQue(std::priority_queue<armorNode> armor)
    {
        armor_ = armor;
    }
    std::priority_queue<armorNode> lightBarThread::getArmorQue()
    {
        std::priority_queue<armorNode> temp;
        {
            std::unique_lock<std::mutex> mutex_(mutex_armor_);
            condition_armor_.wait_for(mutex_, std::chrono::microseconds(10), []()
                                      { return is_armor_getable; });
            temp = armor_;
        }
        return temp;
    }
    void lightBarThread::getGammaCorrection(cv::Mat &src, cv::Mat &dst)
    {
        cv::LUT(src, table, dst);
    }
    void lightBarThread::getGammaTable()
    {
        float fGamma = 1 / gamma;
        table.create(1, 256, CV_8UC1);
        uchar *pointer = table.ptr<uchar>(0);
        for (int i = 0; i < 256; i++)
        {
            *pointer = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
            pointer++;
        }
    }
    std::priority_queue<lightBarNode> lightBarThread::ROI(std::priority_queue<lightBarNode> lightBar) //截取灯条上的ROI
    {
        std::priority_queue<lightBarNode> ROI;
        auto lightBar_copy = lightBar;
        while (!lightBar_copy.empty())
        {
            lightBarNode bestLightBar = lightBar_copy.top();
            bestLightBar.width = bestLightBar.width * width_ceo;
            bestLightBar.length = bestLightBar.length * length_ceo;
            for (auto &i : bestLightBar.point)
            {
                // 计算roi的四个点（可以考虑改进成一个点存储）
                i.x = fmin(fmax(width_ceo * (i.x - bestLightBar.center.x) + bestLightBar.center.x, 0), pic_.mat.rows);
                i.y = fmin(fmax(length_ceo * (i.y - bestLightBar.center.y) + bestLightBar.center.y, 0), pic_.mat.cols);
            }
            ROI.push(bestLightBar);
            lightBar_copy.pop();
        }
        return ROI;
    }
}