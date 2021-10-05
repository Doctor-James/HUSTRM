#ifndef __SCORE_
#define __SCORE_

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "tools.h"
using namespace std;

namespace ly
{
    class circle
    {
    public:
        circle() {}
        double getValue(double x);
        void setCircle(cv::Point2f pt1, cv::Point2f pt2, cv::Point2f pt3);
        cv::Point2d center_;
        cv::Point2d vec;
        double radius_;
    };
    class score
    {
    public:
        score() = default;
        ~score() = default;
        explicit score(param config);
        void resetFunc();
        double getScore(double x);

        circle left_;
        circle right_;
        double setMin_{};
        double setMax_{};
        double setMiddle_{};
        double leftCeo_{};
        double rightCeo_{};

    private:
        void calFunction(circle &C, double middle, double aside, double Ceo);
    };
}

#endif //__SCORE_
