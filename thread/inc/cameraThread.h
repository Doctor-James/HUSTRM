/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-20 12:53:39
 */
#ifndef __CAMERA_THREAD_H
#define __CAMERA_THREAD_H

#include "camera.h"
//#include "HIK_camera.h"
//#include "DAHEN_camera.h"
#include <mutex>
#include <thread>
#include "tools.h"
#include "thread.h"
#include "GxCamera.h"
#include <cstring>

namespace ly
{
    class cameraThread : public camera, public thread
    {
    public:
        cameraThread() = default;
        explicit cameraThread(const cam_device &config, cam_param camera);
        Mat getFrame() override;
        //Mat getFrame2() override;
        int getType() override { return type_; }
        inline double getUpdateTime() override { return cam_->getUpdateTime(); }
        inline int getFrameId() override { return cam_->getFrameId(); }
        double timeSum;
        std::string saveImagePath = "../data/pic"; //保存图片路径
        int imageId;

    private:
        int type_; //相机类型
        bool isOpen = false;
        camera *cam_{};
        GxCamera *cam_new{};
        cam_device device;
        cam_param cameraParam;
        void process();
        time counter_;
    };
}

#endif //__CAMERA_THREAD_H