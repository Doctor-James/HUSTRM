#ifndef __KALMANFLITER_H
#define __KALMANFLITER_H

#include<opencv2/opencv.hpp>
#include <vector>
#include "tools.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.h"
#include "sophus/so3.h"
#include "pthread.h"
#include <algorithm>
#include <math.h>

namespace ly
{
    class KalmanFliter_ly
    {
    public:
        KalmanFliter_ly();
        ~KalmanFliter_ly();
        void set_KF();
    private:
        cv::KalmanFilter KF_;
        bool is_set_x = false;                     //判断是否赋初值
    };

}






#endif //__KALMANFLITER_H
