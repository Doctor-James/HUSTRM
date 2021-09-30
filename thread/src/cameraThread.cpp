/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-22 20:17:46
 */
#include "cameraThread.h"

namespace ly
{

    /**
     * @brief 根据配置文件初始化不同的类
     * @param config 设备配置参数
     */
    cameraThread::cameraThread(const cam_device &config, cam_param camera)
    {
        device = config;
        cameraParam = camera;

        type_ = config.deviceType;

        switch (type_)
        {
        case DAH_CAM:
            cam_new = new GxCamera();
            start(1000);
            break;
        case PIC:
            cam_ = new picture(config.picPath);
            start(10000); //100帧相机
            break;
        case VIDEO:
            cam_ = new video(config.videoPath);
            start(10000); //100帧相机
            break;
        default:
            break;
        }

    }
    /**
     * @brief 多线程获取相机信息
     */
    void cameraThread::process()
    {
        //counter_.countBegin();
        if (type_ == DAH_CAM)
        {
            static bool start_status = false;
            if(!start_status)
            {
                //init camrea lib
                cam_new->initLib();
                //open device SN号
                cam_new->openDevice("KE0200060393");
                //Attention:   (Width-64)%2=0; (Height-64)%2=0; X%16=0; Y%2=0;
                int64_t width = 1280;
                int64_t height = 1024;
                int64_t OffsetX = 0;
                int64_t OffsetY = 0;
                //   ROI             Width  Height        X       Y
                cam_new->setRoiParam(width, height, OffsetX, OffsetY);
                //   ExposureGain          autoExposure  autoGain  ExposureTime  AutoExposureMin  AutoExposureMax  Gain(<=16)  AutoGainMin  AutoGainMax  GrayValue
                cam_new->setExposureGainParam(false, false, 5000, 1000, 3000, 3, 5, 16, 127,true);
                //   WhiteBalance             Applied?       light source type
                cam_new->setWhiteBalanceParam(false, GX_AWB_LAMP_HOUSE_ADAPTIVE);
                //   Acquisition Start!
                cam_new->acquisitionStart(&frame_);
                start_status = true;
            }

            //get image
            cam_new->ProcGetImage((void *)(cam_new->getparam()));

            //cam_new->acquisitionEnd();
        }
        else if (type_ == VIDEO)
        {
            Mat temp = cam_->getFrame();
            {
                std::unique_lock<std::mutex> mutex_(mutex_pic_);
                condition_pic_.wait(mutex_, []()
                                    { return is_pic_getable; });
                frame_ = temp;
            }
        }
         //counter_.countEnd();
         //std::cout << counter_.getTimeMs() << std::endl;
    }
    /**
     * @brief 返回图片信息，相机和非相机两种情况
     * @return 图片信息(灯条识别)
     */
    Mat cameraThread::getFrame()
    {
        Mat temp1;
        {
            std::unique_lock<std::mutex> mutex_(mutex_pic_);
            condition_pic_.wait(mutex_, []()
                                { return is_pic_getable; });
            temp1= frame_;
        }
        return temp1;
    }
    //装甲板ID识别图片
    // Mat cameraThread::getFrame2()
    // {
    //     Mat temp2;
    //     {
    //         std::unique_lock<std::mutex> mutex_(mutex_pic2_);
    //         temp2.mat = frame2_.mat.clone();
    //         temp2.time = frame2_.time;
    //     }
    //     return temp2;
    // }
}
