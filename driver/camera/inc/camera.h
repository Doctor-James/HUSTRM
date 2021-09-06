/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-20 11:59:47
 */

#ifndef __CAMERA__H
#define __CAMERA__H

#include "tools.h"
#include <opencv2/opencv.hpp>

enum type
{
    DAH_CAM = 1,
    PIC = 2,
    VIDEO = 3
};
namespace ly
{
    // 相机，纯虚类
    class camera
    {
    public:
        camera() = default;
        ~camera() = default;
        virtual Mat getFrame() = 0; //获取灯条处理的图片
        //virtual Mat getFrame2() {}  //获取装甲分类的图片
        virtual int getType() = 0;  //获取相机类型

        virtual inline double getUpdateTime() { return update_time_; }
        virtual inline int getFrameId() { return frame_id_; }
        inline void updateId() { frame_id_++; }
        inline void setUpdateTime(double time) { update_time_ = time; }
        Mat frame_; //用于灯条的识别
        //Mat frame2_;         //用于装甲的识别
        time frame_counter_; //计时器

    private:
        static long long int frame_id_;
        double update_time_;
    };
    // 由虚类派生成四个相机类代表海康相机、大恒相机、图片数据和视频数据

    class video : public camera
    {
    public:
        video() = default;
        explicit video(const std::string &video_path);
        Mat getFrame() override;
        int getType() override { return VIDEO; }

    private:
        cv::VideoCapture cap_;
    };
    class picture : public camera
    {
    public:
        picture() = default;
        explicit picture(std::string pic_path);
        Mat getFrame() override;
        int getType() override { return PIC; }

    private:
        std::string pic_path_;
    };
}

#endif //__CAMERA__H
