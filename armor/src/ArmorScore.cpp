#include "ArmorScore.h"
namespace ly
{
    ArmorScore::ArmorScore(armor_param &config)
    {
        config_ = std::move(config);
        LWR_cir_ = new score(config.lengthWidthRadio);
        LBR_cir_ = new score(config.lightBarRadio);
        angle_cir_ = new score(config.angle);
        area_cir_ = new score(config.area);
        parallelism_cir_ = new score(config.parallelism);
        lastCenter_cir_ = new score(config.lastCenter);
        horizontalParallelism_cir_ = new score(config.hParallelism);
    }
    ArmorScore::~ArmorScore()
    {
        delete (LWR_cir_);
        delete (LBR_cir_);
        delete (angle_cir_);
        delete (area_cir_);
        delete (parallelism_cir_);
        delete (lastCenter_cir_);
        delete (horizontalParallelism_cir_);
    }
    void ArmorScore::checkArmor(armorNode &armor_)
    {
        armor_.LBR_ceo = (float)(LBR_cir_->getScore(armor_.lightBarRatio) * config_.LBR_w);
        armor_.LWR_ceo = (float)(LWR_cir_->getScore(armor_.lengthWidthRatio) * config_.LWR_w);
        armor_.angle_ceo = (float)(angle_cir_->getScore(armor_.angle) * config_.angle_w);
        armor_.area_ceo = (float)(area_cir_->getScore(armor_.area) * config_.area_w);
        armor_.parallelism_ceo = (float)(parallelism_cir_->getScore(armor_.parallelism) * config_.parallelism_w);
        armor_.lastCenter_ceo = (float)(lastCenter_cir_->getScore(armor_.deltaCenter) * config_.lastCenter_w);
        armor_.hp_ceo = (float)(horizontalParallelism_cir_->getScore(armor_.horizontalParallelism) * config_.hp_w);
        armor_.sum = armor_.LWR_ceo + armor_.angle_ceo + armor_.area_ceo + armor_.parallelism_ceo + armor_.LBR_ceo + armor_.lastCenter_ceo + armor_.hp_ceo;
        // 开始计算得分
        if (armor_.lengthWidthRatio > 6 || armor_.lengthWidthRatio < 1.5 ||
            armor_.area < 100 ||
            armor_.angle < -20 || armor_.angle > 20 ||
            armor_.parallelism < -10 || armor_.parallelism > 10 ||
            armor_.horizontalParallelism > 0.35)
        {
            armor_.disable = 1;
        }
    }
}