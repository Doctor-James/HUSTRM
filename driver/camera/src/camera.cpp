/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-19 17:58:22
 */
#include "camera.h"
#include <iostream>
#include "tools.h"
#include <opencv2/opencv.hpp>
#include <utility>

namespace ly
{
    long long int camera::frame_id_ = 0;

    /**
     * @brief 
     * @return 
     */

    Mat picture::getFrame()
    {
        setUpdateTime(frame_counter_.getTimeMs());
        std::stringstream ss;
        //        ss<<pic_path_<<"/"<<getFrameId()<<".jpg";
        ss << pic_path_; //
        auto pic = cv::imread(ss.str());
        updateId();
        frame_counter_.countBegin();
        return Mat();
    }
    picture::picture(std::string pic_path)
    {
        pic_path_ = std::move(pic_path);
    }

    /**
     * @brief 获得一帧
     * @return Mat类
     */
    Mat video::getFrame()
    {
        setUpdateTime(frame_counter_.getTimeMs());
        cap_ >> frame_.mat;
        updateId();
        frame_counter_.countBegin();
        return frame_;
    }
    /**
     * @brief 视频类
     * @param video_path 
     */
    video::video(const std::string &video_path)
    {
        cap_.open(video_path);
        if (!cap_.isOpened())
        {
            std::cerr << "error !!! can not open the video!" << std::endl;
            return;
        }
        else
        {
            std::cout << "open video success !" << std::endl
                      << "path:" << video_path << std::endl;
        }
    }
}
