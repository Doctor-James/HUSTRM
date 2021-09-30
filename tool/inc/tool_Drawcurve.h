#ifndef __TOOL_DRAWCURVE_H
#define __TOOL_DRAWCURVE_H
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include <vector>

#define SIN_POINT_NUM 400
namespace ly
{
    class DrawCurve{
    public:
        void ClearSaveData();
        void InsertData(float Data);
        void InsertData(float Data1,float Data2,std::string s1,std::string s2);

    private:
//保留数据
        float SavePoint[SIN_POINT_NUM];         //保存点位
        float SecSavePoint[SIN_POINT_NUM];         //保存点位
        int Times = 0;
    };
}


#endif //__TOOL_DRAWCURVE_H