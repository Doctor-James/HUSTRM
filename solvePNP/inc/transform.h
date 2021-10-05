#ifndef __SOLVE_TRANSFROM_H
#define __SOLVE_TRANSFROM_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "tool_config.h"
#include "node.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include <pangolin/pangolin.h>
#include <algorithm>
#include <lcm/lcm-cpp.hpp>
#include <unistd.h>
#include "eigen3/Eigen/Dense"
#define DEBUG_TRANS
namespace ly
{

    class transform
    {
    public:
        transform();
        Sophus::SE3<double> armor_;
        Sophus::SE3<double> last_armor_;
        void setArmor2Cam(Sophus::SE3<double> armor2cam, receiveData receiveData_);
        //弹道
        Eigen::Vector2f ballistic_equation(float gim_pitch);
        float forward_ballistic_equation(float angle, float x);
        float derivation(float angle, float x);
        float m_set_speed = 27;

        int publish();
        float trajectory(float distance);
        double pitch;
        double yaw;
        double temp;
        sendData sendData_;

    private:
        Sophus::SE3<double> world_;
        Sophus::SE3<double> imu_;
        Sophus::SE3<double> camera_;
        Sophus::SE3<double> gimbal_;
        Sophus::SE3<double> shooter_;
        Sophus::SE3<double> gimbal_to_cam_;
        std::ifstream *fangle;
        std::ifstream *fpitch;
        std::ifstream *fyaw;
        // kalman filter
        // x y z theta_x theta_y theta_z
        cv::Mat measurement_;
        cv::KalmanFilter KF_;
        cv::Mat state_;
        cv::Mat processNoise_;
        cv::Mat KF_prediction = cv::Mat::zeros(4, 1, CV_32F);
        cv::Mat prediction = cv::Mat::zeros(2, 1, CV_32F);
        float delta_t;
        float distance;
        int debug_ = 0;
        time counter_;

#ifdef DEBUG_TRANS
        inline void glDrawColouredCuboid(const Sophus::SE3<double> T, GLfloat a = 0.175f, GLfloat b = 0.004f, GLfloat c = 0.055f);
        static void *visual(void *__this);
        void glDrawColouredAxis(const Sophus::SE3<double> T, float length);
        void setimu(float pitch, float yaw, float roll);
#endif
    };
}

#endif //__SOLVE_TRANSFROM_H
