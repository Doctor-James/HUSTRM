#ifndef _ARMOR_SCORE_H
#define _ARMOR_SCORE_H
#include "score.h"
#include "node.h"
namespace ly
{
    class ArmorScore
    {
    public:
        ArmorScore(armor_param &config);
        ~ArmorScore();
        void checkArmor(armorNode &armor_);

    private:
        armor_param config_;
        score *LWR_cir_;
        score *LBR_cir_;
        score *angle_cir_;
        score *area_cir_;
        score *parallelism_cir_;
        score *lastCenter_cir_;
        score *horizontalParallelism_cir_;
    };

}

#endif