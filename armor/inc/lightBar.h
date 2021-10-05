#ifndef __LIGHT_BAR_
#define __LIGHT_BAR_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "tools.h"
#include <queue>
#include <vector>
#include "node.h"
#include "LightBarScore.h"
#include <ctime>
namespace ly
{
#define CHECK_DISTANCE 10  //控制多远的灯条视为同一个
#define MIN_PREQUE_SCORE 1 //定义最低的前一帧图片分数
    class lightBar
    {
    public:
        lightBar() = default;
        lightBar(lightBar_param config);
        ~lightBar();
        std::priority_queue<lightBarNode> lightBarsQue_;
        void detect(Mat &rawPic, std::priority_queue<lightBarNode> preLBQ);

    private:
        LightBarScore *lightbar_score;
        int colour;
        int thresh_;

        lightBarNode ImageProcess(cv::Mat &rawPic, int is_use_roi = 0, cv::Point2f ROI_Pt_ = cv::Point2f(0, 0));
        void detectByROI(Mat &rawPic, std::priority_queue<ly::lightBarNode> preLBQ);
        void GlobalDetect(Mat &rawPic);
    };
}

#endif //__LIGHT_BAR_
