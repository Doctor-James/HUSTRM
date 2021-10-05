#include "armorDetector.h"
namespace ly
{
    armorDetector::armorDetector()
    {
        config_ = new config();
        camera_ = new cameraThread(config_->getConfigData().camDevice, config_->getConfigData().camParam);
        // serialPortRead_ = new serialPortReadThread(config_->getConfigData().serialPortDev);
        // serialPortWrite_ = new serialPortWriteThread(config_->getConfigData().serialPortDev);
        light_ = new lightBarThread(config_->getConfigData().camParam, config_->getConfigData().lightBarParam);
        armor_ = new armorThread(config_->getConfigData().armorParam, config_->getConfigData().largeArmorParam, config_->getConfigData().camParam);
        wait();
    }
    void armorDetector::wait()
    {
        camera_->join();
        serialPortRead_->join();
        serialPortWrite_->join();
        light_->join();
        camera_->join();
    }
    armorDetector::~armorDetector()
    {
        delete config_;
        delete armor_;
        delete camera_;
        delete light_;
        delete serialPortRead_;
        delete serialPortWrite_;
    }
}
