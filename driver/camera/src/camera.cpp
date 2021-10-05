#include "camera.h"
namespace ly
{
    long long int camera::frame_id_ = 0;
    void picture::getFrame(ly::Mat &output_frame)
    {
        setUpdateTime(frame_counter_.getTimeMs());
        std::stringstream ss;
        //        ss<<pic_path_<<"/"<<getFrameId()<<".jpg";
        ss << pic_path_; //
        output_frame.mat = cv::imread(ss.str());
        output_frame.time = clock();
        updateId();
        frame_counter_.countBegin();
    }
    picture::picture(std::string pic_path)
    {
        pic_path_ = std::move(pic_path);
    }

    /**
     * @brief 获得一帧
     * @return Mat类
     */
    void video::getFrame(ly::Mat &output_frame)
    {
        setUpdateTime(frame_counter_.getTimeMs());
        cap_ >> output_frame.mat;
        output_frame.time = clock();
        updateId();
        frame_counter_.countBegin();
    }
    /**
     * @brief 视频类
     * @param video_path 
     */
    video::video(const std::string &video_path)
    {
        cap_.open(video_path);
        if (!cap_.isOpened())
        {
            std::cerr << "error !!! can not open the video!" << std::endl;
            return;
        }
        else
        {
            std::cout << "open video success !" << std::endl
                      << "path:" << video_path << std::endl;
        }
    }
}
