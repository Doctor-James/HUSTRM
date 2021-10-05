#ifndef __CAMERA_THREAD_H
#define __CAMERA_THREAD_H

#include "camera.h"
#include "tools.h"
#include "thread.h"
#include "GxCamera.h"
#include <cstring>

namespace ly
{
    class cameraThread : public thread
    {
    public:
        cameraThread() = default;
        explicit cameraThread(const cam_device &config, cam_param camera);
        int getType() { return type_; }
        inline double getUpdateTime() { return cam_->getUpdateTime(); }
        inline int getFrameId() { return cam_->getFrameId(); }
        camera *cam_{};

    private:
        int type_; //相机类型
        Mat frame_;
        cam_device device;
        cam_param cameraParam;
        void process();
    };
}

#endif //__CAMERA_THREAD_H