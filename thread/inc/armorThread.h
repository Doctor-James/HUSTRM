#ifndef __ARMOR_THREAD_H
#define __ARMOR_THREAD_H
#include "solvePNP.h"
#include "armor.h"
#include "thread.h"
#include "tools.h"
#define constraint(a, min, max) \
    fmin(fmax(a, min), max) //限制a在min与max之间

namespace ly
{
    class armorThread : public thread
    {
    public:
        armorThread() = default;
        explicit armorThread(armor_param config, armor_param large, cam_param cam_config);
        ~armorThread();

    private:
        void process() override;
        bool verifyArmor(cv::Mat pic, std::priority_queue<armorNode> armor);
        cv::Mat getArmorRoi(armorNode);
        void updateArmor();
        solvePNP *pnp_;
        score *score_;
        bool debug_ = true;    //debug模式
        armorNode best_armor_; //最优装甲板信息
        armor *small_armor;
        std::ofstream FileReader;
    };
}

#endif //__ARMOR_THREAD_H
