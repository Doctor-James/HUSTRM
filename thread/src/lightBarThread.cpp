#include "lightBarThread.h"

namespace ly
{
#define WIDTH_CEO 3
#define LENGTH_CEO 1.5
#define _DEBUG_
    bool debug_show_ROI = false;   //可视化ROI
    bool debug_show_light = false; //可视化lightbar
    bool debug_show_armor = true;  //可视化armor
    lightBarThread::lightBarThread(cam_param cam_config, lightBar_param config)
    {
        lightBar_ = new lightBar(std::move(config));
        start(1000);
    }
    lightBarThread::~lightBarThread()
    {
        delete lightBar_;
    }
    void lightBarThread::process()
    {
        share_->getPic(pic_);
        if (pic_.mat.empty())
        {
            std::cout << "cannot get photo" << std::endl;
            return;
        }
        lightBar_->detect(pic_, preQue_); //在roi中检测lightBar
        share_->setLightbar(lightBar_->lightBarsQue_);
        preQue_ = ROI(lightBar_->lightBarsQue_);
        if (!preQue_.empty())
        {
            std::cout << lightBar_->lightBarsQue_.top().sum << " ";
            std::cout << preQue_.top().sum << std::endl;
        }
        debug_show();
        // counter_.countEnd();
        // std::cout << counter_.getTimeMs() << std::endl;
        // counter_.countBegin();
    }
    void lightBarThread::debug_show()
    {
        if (debug_show_light || debug_show_armor || debug_show_ROI)
        {
            std::priority_queue<ly::armorNode> armor_copy;
            share_->getArmor(armor_copy);
            if (debug_show_light) //显示LightBar
            {
                std::priority_queue<lightBarNode> show_lightBar = lightBar_->lightBarsQue_;
                int times = show_lightBar.size() > 5 ? 5 : (int)show_lightBar.size();
                for (int i = 0; i < times; ++i)
                {
                    if (show_lightBar.empty())
                        continue;
                    cv::Point2f rect_points[4];
                    for (int j = 0; j < 4; j++)
                    {
                        line(pic_.mat, show_lightBar.top().point[j], show_lightBar.top().point[(j + 1) % 4], 255, 2, 8);
                    }
                    std::stringstream ss;
                    ss << show_lightBar.top().angle;
                    std::string num = ss.str();
                    cv::putText(pic_.mat, num, show_lightBar.top().point[0], cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 255, 0));
                    show_lightBar.pop();
                }
            }
            if (!armor_copy.empty() && debug_show_armor) //显示armor
            {
                std::stringstream ss;
                ss << armor_copy.top().lightBarRatio;
                std::string num = ss.str();
                cv::putText(pic_.mat, num, armor_copy.top().center, cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(pic_.mat, "0", armor_copy.top().corner[0], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(pic_.mat, "1", armor_copy.top().corner[1], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(pic_.mat, "2", armor_copy.top().corner[2], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(pic_.mat, "3", armor_copy.top().corner[3], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);

                for (int j = 0; j < 4; j++)
                {
                    line(pic_.mat, armor_copy.top().corner[j], armor_copy.top().corner[(j + 1) % 4], 255, 2, 8);
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
                        line(pic_.mat, show_lightBar.top().point[j], show_lightBar.top().point[(j + 1) % 4], 255, 2, 8);
                    }
                    std::stringstream ss;
                    ss << i; //lightBar.top().angle;
                    std::string num = ss.str();
                    cv::putText(pic_.mat, num, show_lightBar.top().point[0], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                    show_lightBar.pop();
                }
            }
            cv::namedWindow("lightBar debug", 1);
            cv::imshow("lightBar debug", pic_.mat);
            cv::waitKey(1);
        }
    }
    std::priority_queue<lightBarNode> lightBarThread::ROI(std::priority_queue<lightBarNode> lightBar) //截取灯条上的ROI
    {
        std::priority_queue<lightBarNode> ROI;
        while (!lightBar.empty())
        {
            lightBarNode bestLightBar = lightBar.top();
            bestLightBar.width = bestLightBar.width * WIDTH_CEO;
            bestLightBar.length = bestLightBar.length * LENGTH_CEO;
            for (auto &i : bestLightBar.point)
            {
                // 计算roi的四个点（可以考虑改进成一个点存储）
                i.x = fmin(fmax(WIDTH_CEO * (i.x - bestLightBar.center.x) + bestLightBar.center.x, 0), pic_.mat.rows);
                i.y = fmin(fmax(LENGTH_CEO * (i.y - bestLightBar.center.y) + bestLightBar.center.y, 0), pic_.mat.cols);
            }
            ROI.push(bestLightBar);
            lightBar.pop();
        }
        return ROI;
    }
}