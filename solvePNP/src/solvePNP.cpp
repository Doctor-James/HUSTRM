/**
 * @file solvePNP.cpp
 * @brief solve position of armor
 * @author lyc
 */

#include "solvePNP.h"

namespace ly
{
#define SMALL 0
#define LARGE 1
    solvePNP::solvePNP()
    {
    }
    solvePNP::solvePNP(double fx, double fy, double u0, double v0, double k_1, double k_2, double p_1, double p_2,
                       double k_3)
    {
        transform_ = new transform();
        setCameraMatrix(fx, fy, u0, v0);
        setDistortionCoefficients(k_1, k_2, p_1, p_2, k_3);
    }
    solvePNP::solvePNP(ly::cam_param config)
    {
        double fx = config.fx;
        double fy = config.fy;
        double u0 = config.u0;
        double v0 = config.v0;
        double k_1 = config.k_1;
        double k_2 = config.k_2;
        double p_1 = config.p_1;
        double p_2 = config.p_2;
        double k_3 = config.k_3;
        setCameraMatrix(fx, fy, u0, v0);
        setDistortionCoefficients(k_1, k_2, p_1, p_2, k_3);
        transform_ = new transform();
    }
    solvePNP::~solvePNP()
    {
        delete transform_;
    }
    //设置相机内参
    void solvePNP::setCameraMatrix(double fx, double fy, double u0, double v0)
    {
        camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
        camera_matrix.ptr<double>(0)[0] = fx;
        camera_matrix.ptr<double>(0)[2] = u0;
        camera_matrix.ptr<double>(1)[1] = fy;
        camera_matrix.ptr<double>(1)[2] = v0;
        camera_matrix.ptr<double>(2)[2] = 1.0f;
    }
    //设置畸变系数矩阵
    void solvePNP::setDistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3)
    {
        distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
        distortion_coefficients.ptr<double>(0)[0] = k_1;
        distortion_coefficients.ptr<double>(1)[0] = k_2;
        distortion_coefficients.ptr<double>(2)[0] = p_1;
        distortion_coefficients.ptr<double>(3)[0] = p_2;
        distortion_coefficients.ptr<double>(4)[0] = k_3;
    }
    void solvePNP::solve(armorNode armor, int type, receiveData receiveData_)
    {
        Points3D.clear();
        Points2D.clear();
        Points2D.push_back(armor.corner[0]);
        Points2D.push_back(armor.corner[1]);
        Points2D.push_back(armor.corner[2]);
        Points2D.push_back(armor.corner[3]);
        //对2D点归一化到最近邻的平面上
        cv::Mat inliers;
        if (type == SMALL)
        {
            Points3D.emplace_back(cv::Point3f(-0.0650f, 0.0275f, 0.f));
            Points3D.emplace_back(cv::Point3f(0.0650f, 0.0275f, 0.f));
            Points3D.emplace_back(cv::Point3f(0.0650f, -0.0275f, 0.f));
            Points3D.emplace_back(cv::Point3f(-0.0650f, -0.0275f, 0.f));
        }
        else if (type == LARGE)
        {
            Points3D.emplace_back(cv::Point3f(-0.1150f, 0.0275f, 0.f));
            Points3D.emplace_back(cv::Point3f(0.1150f, 0.0275f, 0.f));
            Points3D.emplace_back(cv::Point3f(0.1150f, -0.0275f, 0.f));
            Points3D.emplace_back(cv::Point3f(-0.1150f, -0.0275f, 0.f));
        }

        if (Points3D.size() == 4 || Points2D.size() == 4)
        {

            solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            //cv::solvePnPRansac(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, 200, 8.0, 0.80, inliers, cv::SOLVEPNP_AP3P);
        }
        double temp = rvec.ptr<double>(0)[1];
        rvec.ptr<double>(0)[1] = rvec.ptr<double>(0)[2];
        rvec.ptr<double>(0)[2] = temp;
        temp = tvec.ptr<double>(0)[1];
        tvec.ptr<double>(0)[1] = tvec.ptr<double>(0)[2];
        tvec.ptr<double>(0)[2] = temp;

        //下面是将mat类型的旋转/平移向量转换为SE3型的TF
        cv::Mat R;
        cv::Rodrigues(rvec, R); //旋转矢量转换为旋转矩阵
        Eigen::Matrix3d Rotate_M = Eigen::Matrix3d::Identity();
        cv::cv2eigen(R, Rotate_M);

        Sophus::SO3<double> rotate(Rotate_M);
        Eigen::Vector3d translate(tvec.ptr<double>(0)[0], tvec.ptr<double>(0)[1], -tvec.ptr<double>(0)[2]);
        TF = Sophus::SE3<double>(rotate, translate);
        float distance = sqrt(pow(tvec.ptr<double>(0)[0], 2) + pow(tvec.ptr<double>(0)[1], 2) + pow(tvec.ptr<double>(0)[2], 2));
        float yaw = atan2(tvec.ptr<double>(0)[0], tvec.ptr<double>(0)[1]) * 180 / CV_PI;
        float pitch = atan2(tvec.ptr<double>(0)[2], tvec.ptr<double>(0)[1]) * 180 / CV_PI;
        transform_->setArmor2Cam(TF, armor.receiveData_);
        // if (type)
        //     // printf("type  SMALL\n");
        // else
        //     // printf("type  LARGE\n");
    }
    void solvePNP::setTgc(float pitch, float yaw, float roll) //设置状态转移矩阵
    {
        Eigen::Matrix3d rotation_matrix3;
        rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX());
        cv::Mat temp = cv::Mat::ones(3, 3, CV_8UC1);
        cv::eigen2cv(rotation_matrix3, temp);
        Tgc = (cv::Mat_<double>(3, 4) << temp.at<double>(0, 0), temp.at<double>(0, 1), temp.at<double>(0, 2), 0, //A 状态转移矩阵
               temp.at<double>(1, 0), temp.at<double>(1, 1), temp.at<double>(1, 2), 0,
               temp.at<double>(2, 0), temp.at<double>(2, 1), temp.at<double>(2, 2), 0);
    }
}
