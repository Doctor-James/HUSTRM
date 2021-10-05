#ifndef _LIGHTBAR_SCORE_H
#define _LIGHTBAR_SCORE_H
#include "score.h"
#include "node.h"
namespace ly
{
    class LightBarScore
    {
    public:
        LightBarScore(const lightBar_param &config);
        ~LightBarScore();
        void getLightBarScore(lightBarNode &node);

    private:
        score *LWR_cir_;
        score *angle_cir_;
        score *area_cir_;
        score *dis_cir;
    };
}
#endif