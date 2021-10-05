#include "LightBarScore.h"
namespace ly
{
    LightBarScore::LightBarScore(const lightBar_param &config)
    {
        LWR_cir_ = new score(config.lengthWidthRadio);
        angle_cir_ = new score(config.angle);
        area_cir_ = new score(config.area);
        dis_cir = new score(config.distance);
    }
    LightBarScore::~LightBarScore()
    {
        delete angle_cir_;
        delete LWR_cir_;
        delete area_cir_;
        delete dis_cir;
    }
    void LightBarScore::getLightBarScore(lightBarNode &node)
    {
        node.angle_Ceo = (float)angle_cir_->getScore(node.angle);
        node.LWR_Ceo = (float)LWR_cir_->getScore(node.lengthWidthRatio);
        node.area_Ceo = (float)area_cir_->getScore(node.area);
        node.sum = node.angle_Ceo + node.LWR_Ceo + node.area_Ceo; //计算node的分数
        if (node.sum < 1)
        {
            node.disable = 1;
        }
    }

}