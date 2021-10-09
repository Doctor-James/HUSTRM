#include "lightBar.h"
namespace ly
{
#define FEATURE_NUM 4
    lightBar::lightBar(lightBar_param config)
    {
        thresh_ = config.thresh;
        colour = config.color;
        lightbar_score = new LightBarScore(config);
    }
    lightBar::~lightBar()
    {
        delete lightbar_score;
    }
    void lightBar::detect(Mat &rawPic, std::priority_queue<ly::lightBarNode> preLBQ) //灯条线程处理函数
    {
        // if (preLBQ.empty() || preLBQ.top().sum < MIN_PREQUE_SCORE) //没有前一帧的检测数据或前一帧的检测数据很差
        // {
        //     GlobalDetect(rawPic);
        // }
        // else
        // {
        //     detectByROI(rawPic, preLBQ);
        // }
        GlobalDetect(rawPic);
    }
    void lightBar::GlobalDetect(Mat &rawPic)
    {
        while (!lightBarsQue_.empty())
        {
            lightBarsQue_.pop();
        }
        ImageProcess(rawPic.mat, 0);
    }
    lightBarNode lightBar::ImageProcess(cv::Mat &rawPic, int is_use_roi, cv::Point2f ROI_Pt_)
    {
        std::priority_queue<ly::lightBarNode> temp_lightBarQue_;
        if (rawPic.empty())
        {
            return lightBarNode();
        }
        cv::Mat subtract_dst;         //红蓝通道相减得到的图
        cv::Mat thresh;               //二值化后的单通道图像
        std::vector<cv::Mat> rgb_vec; //三通道的颜色向量
        if (colour == 1)              //需要检测蓝色装甲
        {
            cv::split(rawPic, rgb_vec);
            cv::subtract(rgb_vec[0], rgb_vec[2], subtract_dst);
            cv::inRange(subtract_dst, thresh_, 255, thresh); //thresh_在config里面配置
        }

        else if (colour == 0) //需要检测红色装甲
        {
            cv::split(rawPic, rgb_vec);
            cv::subtract(rgb_vec[2], rgb_vec[0], subtract_dst);
            cv::inRange(subtract_dst, thresh_, 255, thresh);
        }
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //查找图像轮廓
        if (contours.size() < 2)                                                        // 没有识别到灯条的情况，返回一个空的lightBarNode
        {
            return lightBarNode();
        }
        std::vector<cv::RotatedRect> possibleLightBars(contours.size());
        for (unsigned int j = 0; j < contours.size(); ++j) // 遍历每一个轮廓，找到最小矩形框
        {
            lightBarNode node;
            cv::Point2f point[4];
            possibleLightBars.at(j) = cv::minAreaRect(contours[j]);
            bool is_get_angle = false;
            cv::RotatedRect tmp_Elli;
            if (contours[j].size() > 6) // fitEllipse要求点集最少有6个点
            {
                tmp_Elli = cv::fitEllipse(contours[j]); // 返回拟合后的椭圆
                //0-360度转为-90-90度
                if (tmp_Elli.angle < 90)
                {
                    tmp_Elli.angle = -tmp_Elli.angle;
                }
                else
                {
                    tmp_Elli.angle = -tmp_Elli.angle + 180;
                }
                is_get_angle = true;
            }

            if (possibleLightBars.at(j).size.height > possibleLightBars.at(j).size.width)
            {
                if (is_get_angle)
                {
                    node.angle = tmp_Elli.angle;
                }
                else
                {
                    node.angle = -possibleLightBars.at(j).angle;
                }
                node.length = possibleLightBars.at(j).size.height;
                node.width = possibleLightBars.at(j).size.width;
                possibleLightBars.at(j).points(point);
                node.point[0] = point[0] + ROI_Pt_;
                node.point[1] = point[3] + ROI_Pt_; //3->1
                node.point[2] = point[2] + ROI_Pt_;
                node.point[3] = point[1] + ROI_Pt_; //1->3
            }
            else
            {
                if (is_get_angle)
                {
                    node.angle = tmp_Elli.angle;
                }
                else
                {
                    node.angle = -possibleLightBars.at(j).angle - 90;
                }
                node.length = possibleLightBars.at(j).size.width;
                node.width = possibleLightBars.at(j).size.height;
                possibleLightBars.at(j).points(node.point);
                for (int k = 0; k < 4; k++)
                {
                    node.point[k] = node.point[k] + ROI_Pt_;
                }
            }
            possibleLightBars.at(j).center += ROI_Pt_;
            //计算lightbar到图像中心的距离和lightbar的面积
            node.distance = std::sqrt((possibleLightBars.at(j).center.x - rawPic.rows / 2) * (possibleLightBars.at(j).center.x - rawPic.rows / 2) + (possibleLightBars.at(j).center.y - rawPic.cols / 2) * (possibleLightBars.at(j).center.y - rawPic.cols / 2));
            node.area = possibleLightBars.at(j).size.area();
            if (node.width == 0)
            {
                node.lengthWidthRatio = 0;
            }
            else
            {
                node.lengthWidthRatio = node.length / node.width;
            }
            node.center = possibleLightBars.at(j).center;
            lightbar_score->getLightBarScore(node);
            //这里缺少了一个对灯条判断的条件
            if (node.disable == 0)
            {
                temp_lightBarQue_.push(node);
            }
        }
        lightBarsQue_ = temp_lightBarQue_; //记得要去除
        // if (is_use_roi == 0)
        // {
        //     std::cout << "global" << std::endl;
        //     lightBarsQue_ = temp_lightBarQue_;
        // }
        // else
        // {
        //     std::cout << "roi" << std::endl;
        //     return temp_lightBarQue_.top();
        // }
    }
    //根据ROI或者全局检测灯条（1个），返回最优灯条或空灯条
    void lightBar::detectByROI(Mat &rawPic, std::priority_queue<ly::lightBarNode> preLBQ)
    {
        if (rawPic.mat.empty()) //原图为空
        {
            std::cerr << "detect rawPic.empty()" << std::endl;
            return;
        }
        while (!lightBarsQue_.empty())
        {
            lightBarsQue_.pop();
        }
        cv::Point2f lastCenter = cv::Point2f(0, 0);
        int times = preLBQ.size() > 5 ? 5 : (int)preLBQ.size();
        for (int i = 0; i < times; i++)
        {
            auto length = (int)(preLBQ.top().length);
            auto width = (int)(preLBQ.top().width);
            auto useROI = (int)2;
            float distance = (preLBQ.top().center.x - lastCenter.x) * (preLBQ.top().center.x - lastCenter.x) +
                             (preLBQ.top().center.y - lastCenter.y) * (preLBQ.top().center.y - lastCenter.y);
            cv::Point2f center = preLBQ.top().center;
            if (distance < CHECK_DISTANCE) //两帧检测到的灯条位置接近，看成一个灯条
            {
                preLBQ.pop();
                continue;
            }
            float roi_left_x = (center.x - length / 2.0f * useROI) < 1 ? 1 : (center.x - length / 2.0f * useROI);
            float roi_left_y = (center.y - width / 2.0f * useROI) < 1 ? 1 : (center.y - width / 2.0f * useROI);
            cv::Point2f ROI_Pt_ = cv::Point2f(roi_left_x, roi_left_y);
            length = (int)(length * useROI + ROI_Pt_.x) > rawPic.mat.cols ? rawPic.mat.cols : (int)(length * useROI + ROI_Pt_.x);
            width = (int)(width * useROI + ROI_Pt_.y) > rawPic.mat.rows ? rawPic.mat.rows : (int)(width * useROI + ROI_Pt_.y);
            cv::Mat ROI = rawPic.mat(cv::Range((int)ROI_Pt_.y, width), cv::Range((int)ROI_Pt_.x, length));
            lastCenter = preLBQ.top().center;
            lightBarsQue_.push(ImageProcess(ROI, 1, ROI_Pt_));
            preLBQ.pop();
        }
    }
}
