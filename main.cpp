#include "main.h"

using namespace ly;
using namespace std;
void getVideo();
int main()
{
    armorDetector *armorDetector_ = new armorDetector();
    // getVideo();
    return 0;
}
//保存视频
void getVideo()
{
    std::string path = "../pn3.avi";
    cv::VideoWriter output(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 50, cv::Size(1280, 1024));

    config *config_ = new config();
    cameraThread *camera_ = new cameraThread(config_->getConfigData().camDevice, config_->getConfigData().camParam);
    cv::waitKey(1000);
    Mat frame;
    while (1)
    {
        camera_->share_->getPic(frame);
        if (frame.mat.empty())
        {
            continue;
        }
        cv::imshow("frame", frame.mat);
        output.write(frame.mat);
        if (cv::waitKey(10) == 'q')
            break;
    }
    output.release();
    delete config_;
    delete camera_;
}
