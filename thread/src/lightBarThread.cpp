/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2021-07-12 20:12:13
 * @LastEditors: Andy
 * @LastEditTime: 2021-07-22 10:49:15
 */
#include <tools.h>
#include "preProcessThread.h"
#include "lightBarThread.h"
//#define _DEBUG_
//#define RGB_DEBUG_
//#define HSV_DEBUG_
#define PIC_DELAY 0  //图像传输时间
namespace ly
{

int last_time;
    lightBarThread::lightBarThread(cam_param cam_config, lightBar_param config, cameraThread *camera, preProcessThread *preProcess, serialPortReadThread *serialPortRead, serialPortWriteThread *serialPortWrite) : lightBar(config), camera_(camera), preProcess_(preProcess), serialPortRead_(serialPortRead), serialPortWrite_(serialPortWrite)
    {
        gamma = config.gamma;
        colour = 0;//config.color;
        init(std::move(config));
        getGammaTable();
        start(0);
    }
    void lightBarThread::process()
    {
        //时间同步
        pic_ = camera_->getFrame();
        if(pic_.mat.empty())
        {return;}
        if(serialPortRead_->getReceiveMsg().empty())
        {return;}
        receive_Data = serialPortRead_->getReceiveMsg();
        pic_.time = (int) (1000000 * (pic_.timeval.tv_sec-receive_Data.at(49).TX2_BeginTime.tv_sec) + (pic_.timeval.tv_usec-receive_Data.at(49).TX2_BeginTime.tv_usec)) / 1000;// -receive_Data.at(49).delta_t-PIC_DELAY;
	if(last_time != pic_.time)
	{
		last_time = pic_.time;

		std::cout<<"Pic_time: "<<pic_.time<<std::endl;
	}		
//std::cout<<receive_Data.at(49).TX2_BeginTime.tv_usec<<std::endl;
        for(int i = 49;i>0;i--)
        {
            if(abs(pic_.time - receive_Data.at(i-1).TX2_Time)>abs((pic_.time - receive_Data.at(i).TX2_Time)))
		{
	//	std::cout<<(pic_.time - receive_Data.at(i).TX2_Time)<<std::endl;
                pic_.receiveData_ = receive_Data.at(i);
                break;
            }
        }

        colour = 0;
//            cv::Mat temp = pic_.mat.clone();
//            float fGamma = 1/2.2;
//            getGammaCorrection(temp,pic_.mat, fGamma);
        if (pic_.mat.empty())
            return;
#ifdef HSV_DEBUG_
        cv::Mat imgHSVDebug;                           //红蓝通道相减得到的图
        cv::Mat threshDebug;                                 //二值化后的单通道>图像
        cv::Mat element;
        cv::cvtColor(pic_.mat, imgHSVDebug, cv::COLOR_BGR2HSV); //BGR转换为HSV空间
        std::vector<cv::Mat> hsvSplitDebug;
        cv::split(imgHSVDebug, hsvSplitDebug);
        cv::equalizeHist(hsvSplitDebug[2], hsvSplitDebug[2]);
        cv::merge(hsvSplitDebug, imgHSVDebug);
        inRange(imgHSVDebug, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), threshDebug); //Threshold the image

        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        //dilate(imgThresholded,imgThresholded,element);
        cv::morphologyEx(threshDebug, threshDebug, cv::MORPH_OPEN, element);  //开操作
        cv::namedWindow("imgThresholded",0);
        cv::createTrackbar( "iLowH : ","imgThresholded", &iLowH, 255, NULL);
        cv::createTrackbar( "iLowS : ","imgThresholded", &iLowS, 255, NULL);
        cv::createTrackbar( "iLowV : ","imgThresholded", &iLowV, 255, NULL);
        cv::createTrackbar( "iHighH : ","imgThresholded", &iHighH, 255, NULL);
        cv::createTrackbar( "iHighS : ","imgThresholded", &iHighS, 255, NULL);
        cv::createTrackbar( "iHighV : ","imgThresholded", &iHighV, 255, NULL);
        imshow("imgThresholded",threshDebug);
        cv::waitKey(10);

#endif


#ifdef RGB_DEBUG_
	int colour =0;
        cv::Mat subtract_dst;                           //红蓝通道相减得到的图
        cv::Mat thresh;                                 //二值化后的单通道图像
        std::vector<cv::Mat> rgb_vec;                   //三通道的颜色向量
        std::vector<std::vector<cv::Point> > contours;  //轮廓点集
        time lightBar_time;                             //计时器
	int thresh_ = 80;
        lightBar_time.countBegin();

        if(colour == 1)                     //需要检测蓝色装甲
        {
            cv::split(pic_.mat,rgb_vec);
            cv::subtract(rgb_vec[0], rgb_vec[2], subtract_dst);
            cv::threshold(subtract_dst, thresh, thresh_, 255, cv::THRESH_BINARY );      //thresh_在config里面配置
        }
        else if (colour == 0)                //需要检测红色装甲
        {
            cv::split(pic_.mat,rgb_vec);
            cv::subtract(rgb_vec[2], rgb_vec[0], subtract_dst);
            cv::threshold(subtract_dst, thresh, thresh_, 255, cv::THRESH_BINARY );
        }

        cv::namedWindow("imgThresholded",0);
        cv::createTrackbar( "thresh_ : ","imgThresholded", &thresh_, 255, NULL);
        imshow("imgThresholded",thresh);
        cv::waitKey(10);

#endif

        getPreQue();                            //获取预处理得到的前一帧的roi
        detect(pic_, preQue_,colour);        //在roi中检测lightBar
        updatePreQue();                         //用检测到的lightBar更新roi
        mutex_lightBar_.lock();
        copy_lightBarsQue_ = lightBarsQue_;     //更新lightBar
        mutex_lightBar_.unlock();
        update_ = true;
#ifdef _DEBUG_
        debug_show();
#endif
    }
    void lightBarThread::getPreQue() //通过预处理线程获取ROI
    {
        if (preProcess_->checkUpdate())
        {
            preQue_ = preProcess_->getLightBarQue(); //这里实际上返回的是roi
        }
    }
    void lightBarThread::updatePreQue() //更新ROI区域
    {
        preQue_ = ROI(lightBarsQue_); //这里实际上返回的是roi
    }
    void lightBarThread::debug_show()
    {
        // lightBar_time.countEnd();
        // std::cout << 1 / lightBar_time.getTimeMs() * 1000 << std::endl;
        // lightBar_time.countBegin();
        if (debug_show_light || debug_show_armor || debug_show_ROI)
        {
            show_ = pic_.mat.clone();
            std::priority_queue<ly::armorNode> armor_copy = getArmorQue();
            if (debug_show_light) //显示LightBar
            {
                std::priority_queue<lightBarNode> show_lightBar = lightBarsQue_;
                int times = show_lightBar.size() > 5 ? 5 : (int)show_lightBar.size();
                for (int i = 0; i < times; ++i)
                {
                    if (show_lightBar.empty())
                        continue;
                    cv::Point2f rect_points[4];
                    for (int j = 0; j < 4; j++)
                    {
                        line(show_, show_lightBar.top().point[j], show_lightBar.top().point[(j + 1) % 4], 255, 2, 8);
                    }
                    std::stringstream ss;
                    ss << show_lightBar.top().angle;
                    std::string num = ss.str();
                    cv::putText(show_, num, show_lightBar.top().point[0], cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 255, 0));
                    show_lightBar.pop();
                }
            }
            if (!armor_copy.empty() && debug_show_armor) //显示armor
            {
                std::stringstream ss;
                ss << armor_copy.top().lightBarRatio;
                std::string num = ss.str();
                cv::putText(show_, num, armor_copy.top().center, cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(show_, "0", armor_copy.top().corner[0], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(show_, "1", armor_copy.top().corner[1], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(show_, "2", armor_copy.top().corner[2], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                cv::putText(show_, "3", armor_copy.top().corner[3], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);

                for (int j = 0; j < 4; j++)
                {
                    line(show_, armor_copy.top().corner[j], armor_copy.top().corner[(j + 1) % 4], 255, 2, 8);
                }
            }
            if (debug_show_ROI) //显示ROI
            {
                std::priority_queue<lightBarNode> show_lightBar = preQue_;

                int times = show_lightBar.size() > 5 ? 5 : (int)show_lightBar.size();
                for (int i = 0; i < times; ++i)
                {
                    if (show_lightBar.empty())
                        continue;
                    cv::Point2f rect_points[4];
                    for (int j = 0; j < 4; j++)
                    {
                        line(show_, show_lightBar.top().point[j], show_lightBar.top().point[(j + 1) % 4], 255, 2, 8);
                    }
                    std::stringstream ss;
                    ss << i; //lightBar.top().angle;
                    std::string num = ss.str();
                    cv::putText(show_, num, show_lightBar.top().point[0], cv::FONT_HERSHEY_COMPLEX, 0.8, 255);
                    show_lightBar.pop();
                }
            }
            cv::namedWindow("lightBar debug", 0);
            cv::imshow("lightBar debug", show_);
            cv::waitKey(30);
        }
    }
    std::priority_queue<lightBarNode> lightBarThread::getLightBarQue() //外部获取灯条接口
    {
//        std::priority_queue<lightBarNode> temp;
//        {
//            std::unique_lock<std::mutex> mutex_(mutex_lightBar_);
//            condition_lightBar_.wait(mutex_, []()
//                                     { return is_lightBar_getable; });
//            temp = copy_lightBarsQue_;
//        }
//        is_lightBar_getable = false;
//        return temp;
        if(update_ && !copy_lightBarsQue_.empty())
        {
            update_ = false;
            mutex_lightBar_.lock();
            auto temp = copy_lightBarsQue_;
            mutex_lightBar_.unlock();
            return temp;
        }
        return std::priority_queue<lightBarNode>();
    }
    void lightBarThread::update_lightBar(std::priority_queue<lightBarNode> node) //生产者
    {
        {
            std::unique_lock<std::mutex> mutex_(mutex_lightBar_);
            is_lightBar_getable = false;
            copy_lightBarsQue_ = node;
        }
        is_lightBar_getable = true;
        condition_lightBar_.notify_one();
    }
    void lightBarThread::setArmorQue(std::priority_queue<armorNode> armor)
    {
        armor_ = armor;
    }
    std::priority_queue<armorNode> lightBarThread::getArmorQue()
    {
        std::priority_queue<armorNode> temp;
        {
            std::unique_lock<std::mutex> mutex_(mutex_armor_);
            condition_armor_.wait_for(mutex_, std::chrono::microseconds(10), []()
                                      { return is_armor_getable; });
            temp = armor_;
        }
        return temp;
    }
    void lightBarThread::getGammaCorrection(cv::Mat &src, cv::Mat &dst)
    {
        cv::LUT(src, table, dst);
    }
    void lightBarThread::getGammaTable()
    {
        float fGamma = 1 / gamma;
        table.create(1, 256, CV_8UC1);
        uchar *pointer = table.ptr<uchar>(0);
        for (int i = 0; i < 256; i++)
        {
            *pointer = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
            pointer++;
        }
    }
    std::priority_queue<lightBarNode> lightBarThread::ROI(std::priority_queue<lightBarNode> lightBar) //截取灯条上的ROI
    {
        std::priority_queue<lightBarNode> ROI;
        auto lightBar_copy = lightBar;
        while (!lightBar_copy.empty())
        {
            lightBarNode bestLightBar = lightBar_copy.top();
            bestLightBar.width = bestLightBar.width * width_ceo;
            bestLightBar.length = bestLightBar.length * length_ceo;
            for (auto &i : bestLightBar.point)
            {
                // 计算roi的四个点（可以考虑改进成一个点存储）
                i.x = fmin(fmax(width_ceo * (i.x - bestLightBar.center.x) + bestLightBar.center.x, 0), pic_.mat.rows);
                i.y = fmin(fmax(length_ceo * (i.y - bestLightBar.center.y) + bestLightBar.center.y, 0), pic_.mat.cols);
            }
            ROI.push(bestLightBar);
            lightBar_copy.pop();
        }
        return ROI;
    }
}
