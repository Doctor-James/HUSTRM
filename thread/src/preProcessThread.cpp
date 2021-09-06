/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-22 10:51:35
 */
#include <tools.h>
#include "preProcessThread.h"

namespace ly
{
    preProcessThread::preProcessThread(lightBar_param config, cameraThread *camera, serialPortReadThread *serialPortRead, serialPortWriteThread *serialPortWrite) : camera_(camera), serialPortRead_(serialPortRead), serialPortWrite_(serialPortWrite)
    {
        init(std::move(config)); //继承自lightar
        colour = 0;//config.color;
        start(10000); //100hz
    }

    void preProcessThread::process()
    {
        pic_ = camera_->getFrame();
        if (pic_.mat.empty())
        {
            return;
        }
#ifdef _USE_SERIALPORT_
        if (serialPortRead_->getReceiveMsg().empty())
        {
            return;
        }
        receive_Data = serialPortRead_->getReceiveMsg();
        time_t end = receive_Data.at(149).Data_Time;
        double t_ms = double(end - pic_.time) / CLOCKS_PER_SEC * 1000;
        std::cout << "time: " << t_ms << std::endl;
        t_ms = fmax(0, fmin(t_ms, 149)); //限制这个时间有什么用？？
        if (receive_Data.end()->flag == 0x05)
        {
            colour = 0; //red
        }
        else if (receive_Data.end()->flag == 0x06)
        {
            colour = 1; //blue
        }
#endif
        detect(pic_, colour);                                    //全图检测灯条
        std::priority_queue<lightBarNode> roi_ = ROI(temp_LBQ_); //根据轮廓来圈出ROI区域
        {
            std::unique_lock<std::mutex> mutex_(mutex_ROI_); //生产ROI
            is_ROI_getable = false;
            ROI_ = roi_;
        }
        update_ = true;
        is_ROI_getable = true;
        condition_ROI_.notify_one();
    }
    std::priority_queue<lightBarNode> preProcessThread::getLightBarQue() //消费者接口
    {
        std::priority_queue<lightBarNode> roi;
        {
            std::unique_lock<std::mutex> mutex_(mutex_ROI_);
            condition_ROI_.wait(mutex_, []()
                                { return is_ROI_getable; });
            roi = ROI_;
        }
        return roi;
    }
    bool preProcessThread::checkUpdate() //检测ROI是否已经更新
    {
        if (update_)
        {
            update_ = false;
            return true;
        }
        else
        {
            return false;
        }
    }
    //将轮廓区域大一定倍数，作为灯条处理的ROI
    std::priority_queue<lightBarNode> preProcessThread::ROI(std::priority_queue<lightBarNode> lightBar) //截取灯条上的ROI
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
