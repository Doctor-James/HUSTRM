#include "armorThread.h"

namespace ly
{
#define SMALL 0
    armorThread::armorThread(armor_param config, armor_param large, cam_param cam_config)
    {
        score_ = new score();
        pnp_ = new solvePNP(cam_config);
        small_armor = new armor(config);
        start(15000);
    }

    armorThread::~armorThread()
    {
        delete score_;
        delete pnp_;
        delete small_armor;
    }

    void armorThread::process()
    {
        static int i = 0;
        std::priority_queue<lightBarNode> lightBar;
        share_->getLightbar(lightBar);
        if (lightBar.size() > 1) //有两个以上灯条
        {
            small_armor->matchLightBar(lightBar); //从检测到的lightBar中检测armor
            if (small_armor->is_armor_empty())
            {
                // std::cout << "cannot get armor" << std::endl;
                return;
            }
            armorNode bestArmor = small_armor->getArmor().top();
            if (bestArmor.sum > 4.5)
            {
                i++;
                pnp_->solve(bestArmor, SMALL, bestArmor.receiveData_);
                // std::cout << "get armor" << i << std::endl;
            }
            //解算失败，直接清空装甲
            else
            {
                small_armor->emptyArmor();
            }
        }

        if (debug_ && !small_armor->is_armor_empty())
        {
            share_->setArmor(small_armor->getArmor());
        }
    }
}
