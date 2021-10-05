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
        std::cout << type_ << std::endl;

        switch (type_)
        {
        case DAH_CAM:
            cam_ = new GxCamera();
            start(10000);
            break;
        case PIC:
            cam_ = new picture(config.picPath);
            start(10000); //100帧相机
            break;
        case VIDEO:
            cam_ = new video(config.videoPath);
            start(5000); //100帧相机
            break;
        default:
            break;
        }
    }
    /**
     * @brief 多线程获取相机信息
     */
    void cameraThread::process() //生产者
    {
        cam_->getFrame(this->frame_);
        share_->setPic(this->frame_);
        // counter_.countEnd();
        // std::cout << counter_.getTimeMs() << std::endl;
        // counter_.countBegin();
    }
}