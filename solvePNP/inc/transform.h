#ifndef __SOLVE_TRANSFROM_H
#define __SOLVE_TRANSFROM_H

#include<opencv2/opencv.hpp>
#include <vector>
#include "tools.h"
#include "armor.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.h"
#include "sophus/so3.h"
#include "pthread.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <pangolin/pangolin.h>
#include "serialPortReadThread.h"
#include "serialPortWriteThread.h"
#include <algorithm>
#include <math.h>
#define DEBUG_TRANS
namespace ly
{

    //击打缓冲计算返回
    typedef struct{
        float pitch;       //rad
        float yaw;         //rad
        float time;        //击打弹道时间(ms)
        float distance;
    } Angle_t;

    typedef struct{
        float pitch;
        float yaw;
        float ShootSpeed = 15;
        double BeginToNowTime = 0;
    } carPose;


    class transform
    {
    public:
        transform(serialPortReadThread *serialPortRead,serialPortWriteThread *serialPortWrite);
        Sophus::SE3 armor_now;
        Sophus::SE3 armor_pre;
        Sophus::SE3 last_armor_;
        void setArmor2Cam(Sophus::SE3 armor2cam,receiveData receiveData_);

        //弹道
        Angle_t ballistic_equation(float gim_pitch,cv::Point3f armor_Position);
        float forward_ballistic_equation(float angle,float x);
        float derivation(float angle,float x);

        //kalman filter
        cv::Point3f set_KF(cv::Point3f Position_now);
        void First_Filter(cv::Point3f Position_now);
        void Continuous_Filter(cv::Point3f Position_now);
        sendData sendData_;
    private:
        serialPortReadThread *serialPortRead_;
        serialPortWriteThread *serialPortWrite_;
        Sophus::SE3 world_;
        Sophus::SE3 imu_;
        Sophus::SE3 camera_;
        Sophus::SE3 gimbal_;
        Sophus::SE3 shooter_;
        Sophus::SE3 gimbal_to_cam_;
        Angle_t shootAngleTime_now;
        Angle_t shootAngleTime_pre;
        carPose carPose_now;
        carPose carPose_old;
        carPose carPose_;
        float i_debug =1;

        // kalman filter
        // x y z theta_x theta_y theta_z
        cv::KalmanFilter KF_;
        cv::Mat measurement_;
        bool is_nfirst_KF = false;
        bool first_find_temp = false;
        cv::Point3f Position_old;
        cv::Point3f Position_pre;
        cv::Point3f v_old;
        cv::Point3f v_pre;

        int debug_ = 0;
        time counter_;
        DrawCurve DrawCurve_;

#ifdef DEBUG_TRANS
        inline void glDrawColouredCuboid(const Sophus::SE3 T, GLfloat a = 0.175f,GLfloat b= 0.004f,GLfloat c=0.055f);
        static void* visual(void* __this);
        void glDrawColouredAxis(const Sophus::SE3 T,float length);
        void setimu(float pitch, float yaw, float roll);
#endif
    };
}






#endif //__SOLVE_TRANSFROM_H
