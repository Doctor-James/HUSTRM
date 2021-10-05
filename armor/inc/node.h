#ifndef _NODE_H
#define _NODE_H
#include <opencv2/opencv.hpp>
#include "tools.h"
namespace ly
{
    typedef struct armorNode
    {
        float lengthWidthRatio;      //长宽比
        float lightBarRatio;         //两灯条长度之比，限制小于1
        float angle;                 //角度
        float area;                  //面积
        float distance;              //离光心的像素平面上的欧式距离
        float parallelism;           //平行度
        float horizontalParallelism; //水平方向平行度

        float LWR_ceo;
        float LBR_ceo;
        float angle_ceo;
        float area_ceo;
        float parallelism_ceo;
        float lastCenter_ceo;
        float hp_ceo;

        float length;
        float width;
        float deltaCenter;     //当前中心和上一帧中心的欧式距离
        int disable = 0;       //装甲板是否可用
        cv::Point2f corner[4]; //armor的四个角，0-3分别为左上，右上，右下，左下
        cv::Point2f center;
        float sum; //总分
        receiveData receiveData_;

        friend bool operator<(armorNode a, armorNode b)
        {
            return a.sum < b.sum;
        }
    } armorNode;
    typedef struct lightBarNode
    {
        float lengthWidthRatio = 0;
        float angle = 0;
        float area = 0;
        float distance = 0;
        float LWR_Ceo = 0;
        float angle_Ceo = 0;
        float area_Ceo = 0;
        float length = 0;
        float width = 0;
        int disable = 0;
        cv::Point2f point[4] = {};
        cv::Point2f center{};
        float sum = 0;
        friend bool operator<(lightBarNode a, lightBarNode b) //重载小于号,便于利用优先级队列排序
        {
            return a.sum < b.sum;
        }
        lightBarNode() {}
    } lightBarNode;

}

#endif