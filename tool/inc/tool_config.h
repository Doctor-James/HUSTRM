#ifndef __TOOL_CONFIG_H
#define __TOOL_CONFIG_H

#include "jsoncpp/json/json.h"
#include <string>
#include <memory>
#include <fstream>
#include <iostream>
namespace ly
{
    typedef struct cam_device
    {
        int deviceType;
        std::string sn;
        std::string videoPath;
        std::string picPath;
        int picNum;
    } cam_device;
    typedef struct serialPort_dev
    {
        bool enable;
        std::string deviceName;
    } serialPort_dev;
    typedef struct cam_param
    {
        double fx;
        double fy;
        double u0;
        double v0;
        double k_1;
        double k_2;
        double p_1;
        double p_2;
        double k_3;
        double ExposureTime; //曝光时间
        double Gain;         //通道增益
    } cam_param;

    typedef struct robot_param
    {
        double gimbal_cam[3];
    } robot_param;
    typedef struct param
    {
        bool enable;
        double best;
        double min;
        double max;
        double left_ceo;
        double right_ceo;
    } param;
    typedef struct lightBar_param
    {
        int thresh;
        int color;
        param lengthWidthRadio;
        param angle;
        param area;
        param distance;
    } lightBar_param;

    typedef struct armor_param //装甲板总参数
    {
        param lengthWidthRadio;
        param lightBarRadio;
        param angle;
        param area;
        param parallelism;
        param lastCenter;
        param hParallelism;
        double LWR_w; //比重
        double angle_w;
        double area_w;
        double distance_w;
        double parallelism_w;
        double LBR_w;
        double lastCenter_w;
        double hp_w;
    } armor_param;
    typedef struct configData //配置信息总类
    {
        cam_device camDevice;
        serialPort_dev serialPortDev;
        cam_param camParam;
        robot_param robotParam;
        lightBar_param lightBarParam;
        armor_param armorParam;
        armor_param largeArmorParam;
    } configData;

    class config
    {
    public:
        config();
        param getParam(const std::string &paramName1, const std::string &paramName2); //获得灯条或者装甲参数
        configData getConfigData() { return configDataSt_; }                          //获得整个配置文件

    private:
        configData configDataSt_;
        Json::CharReaderBuilder builder; //读取json文件类
        Json::Value root;                //Json各标签
        std::string path_ = "../config/armorDetector.json";
    };
}
#endif //__TOOL_CONFIG_H