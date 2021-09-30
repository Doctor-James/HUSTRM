#include "armorDetector.h"
#include<cmath>
#include<iostream>
//#define testSerialPort
//std::vector<receiveData> re_;
namespace ly
{
    armorDetector::armorDetector()
    {
        config_ = new config();
        camera_ = new cameraThread(config_->getConfigData().camDevice, config_->getConfigData().camParam);
        serialPortRead_ = new serialPortReadThread(config_->getConfigData().serialPortDev);
        serialPortWrite_ = new serialPortWriteThread(config_->getConfigData().serialPortDev);
        preProcess_ = new preProcessThread(config_->getConfigData().lightBarParam,camera_, serialPortRead_, serialPortWrite_);
        light_ = new lightBarThread(config_->getConfigData().camParam,config_->getConfigData().lightBarParam,camera_,preProcess_,serialPortRead_,serialPortWrite_);
        armor_ = new armorThread(config_->getConfigData().armorParam, config_->getConfigData().largeArmorParam, config_->getConfigData().camParam, light_, serialPortRead_, serialPortWrite_, camera_);

#ifdef testSerialPort
	config_ = new config();
        serialPortRead_ = new serialPortReadThread(config_->getConfigData().serialPortDev);
        serialPortWrite_ = new serialPortWriteThread(config_->getConfigData().serialPortDev);
	int count = 0;
        while(1){
		
           re_ = serialPortRead_->getReceiveMsg();
//            std::cout <<  "p: " <<  re_.pitch <<
//                      "y: " <<  re_.yaw   <<
//                      std::endl;
        if(re_.at(49).yaw==3)
{
		sendData se_;
count++;
if(count>10000)
{
continue;
}        
    se_.yaw = count;
	
        std::cout<<"  "<<se_.yaw<<std::endl;   
	 serialPortWrite_->setSendMsg(se_);
}    


            usleep(1);
        }
#endif

    }
    armorDetector::~armorDetector()
    {
        delete config_;
        delete armor_;
        delete camera_;
        delete light_;
        delete preProcess_;
//	    delete serialPortRead_;
//        delete serialPortWrite_;
	    //delete transform_;
    }
}
