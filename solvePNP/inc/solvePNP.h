#ifndef __SOLVE_PNP_H
#define __SOLVE_PNP_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "tool_config.h"
#include "node.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include "transform.h"
#include "opencv2/core/eigen.hpp"
#include <math.h>
#include <fstream>

namespace ly
{
    class solvePNP
    {
    public:
        solvePNP();
        solvePNP(cam_param config);
        solvePNP(double fx, double fy, double u0, double v0, double k_1, double k_2, double p_1, double p_2, double k_3);
        ~solvePNP();
        void solve(ly::armorNode armor, int type, receiveData receiveData_);
        void setCameraMatrix(double fx, double fy, double u0, double v0);
        void setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3);
        void setTgc(float pitch, float yaw, float roll);
        Sophus::SE3<double> TF;
        cv::Mat TF_Mat;
        cv::Mat rvec; //解出来的旋转向量
        cv::Mat tvec; //解出来的平移向量
        cv::Mat Tgc;
        time time_counter_;
        double times;

    private:
        std::vector<cv::Point3f> Points3D; //存储四个点的世界坐标
        std::vector<cv::Point2f> Points2D; //存储四个点的图像坐标

        cv::Mat camera_matrix;           //内参数矩阵
        cv::Mat distortion_coefficients; //畸变系数
        transform *transform_;
    };
}

#endif //__SOLVE_PNP_H
