#ifndef __ARMOR_
#define __ARMOR_

#include <opencv2/opencv.hpp>
#include "node.h"
#include "ArmorScore.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace ly
{
    class armor
    {
    public:
        armor() = default;
        armor(armor_param config);
        ~armor();
        void matchLightBar(std::priority_queue<lightBarNode> lightBars);
        std::priority_queue<armorNode> getArmor() { return armor_; }
        bool is_armor_empty() { return armor_.empty(); };
        void emptyArmor();

    protected:
        void judgeArmor(const lightBarNode &lightBar_1, const lightBarNode &lightBar_2);
        std::priority_queue<armorNode> armor_; //装甲队列
        armorNode bestArmor_;                  //最优装甲
        ArmorScore *armor_score;               //计算得分
        cv::Point2f last_center;               //上一个装甲的中心
    };
}

#endif //__ARMOR_
