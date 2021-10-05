#include "armor.h"

namespace ly
{
    armor::armor(armor_param config)
    {
        armor_score = new ArmorScore(config);
    }
    armor::~armor()
    {
        delete armor_score;
    }
    void armor::emptyArmor()
    {
        while (!armor_.empty())
        {
            armor_.pop();
        }
    }
    void armor::matchLightBar(std::priority_queue<lightBarNode> lightBars) //配对灯条，找到最优装甲板
    {
        if (lightBars.empty() || lightBars.top().sum < 1.5) //没有检测到lightBar，或者效果太差
        {
            return;
        }
        while (!armor_.empty()) //清空armor_，用lightBar检测新的armor_
            armor_.pop();
        int size1 = fmin(4, lightBars.size()); //最多留4个灯条匹配
        while (size1 >= 0 && !lightBars.empty())
        {
            lightBarNode lightBar1 = lightBars.top();
            lightBars.pop();
            std::priority_queue<lightBarNode> lightBars_bak(lightBars);

            int size2 = fmin(3, lightBars.size());
            while (size2 >= 0 && !lightBars_bak.empty())
            {
                lightBarNode lightBar2 = lightBars_bak.top();
                lightBars_bak.pop();
                judgeArmor(lightBar1, lightBar2); //灯条配对
                size2--;
            }
            size1--;
        }
        if (armor_.size() == 1) //如果仅仅只有一个的话，那么就是最优装甲
        {
            bestArmor_ = armor_.top();
        }
        else if (armor_.size() >= 2) //否则需要判断@TODO
        {
        }

        if (!armor_.empty())
        {
            last_center = armor_.top().center;
        }
    }
    /**
     * @brief 两个灯条节点是否能构成一个装甲,计算特征
     * @param lightBar_1
     * @param lightBar_2
     */
    void armor::judgeArmor(const lightBarNode &lightBar_1, const lightBarNode &lightBar_2)
    {
        armorNode armor;
        armor.center = (lightBar_1.center + lightBar_2.center) / 2;
        cv::Point2f point[4];
        point[0] = (lightBar_1.point[0] + lightBar_1.point[1]) / 2;
        point[1] = (lightBar_1.point[2] + lightBar_1.point[3]) / 2;
        point[2] = (lightBar_2.point[0] + lightBar_2.point[1]) / 2;
        point[3] = (lightBar_2.point[2] + lightBar_2.point[3]) / 2;
        if (point[0].x > point[2].x)
        {
            if (point[0].y < point[1].y) //point[0]是右上角点,point[1]是右下角点
            {
                armor.corner[1] = point[0];
                armor.corner[2] = point[1];
            }
            else
            {
                armor.corner[1] = point[1];
                armor.corner[2] = point[0];
            }
            if (point[2].y < point[3].y)
            {
                armor.corner[0] = point[2];
                armor.corner[3] = point[3];
            }
            else
            {
                armor.corner[0] = point[3];
                armor.corner[3] = point[2];
            }
        }
        else
        {
            if (point[0].y < point[1].y)
            {
                armor.corner[0] = point[0];
                armor.corner[3] = point[1];
            }
            else
            {
                armor.corner[0] = point[1];
                armor.corner[3] = point[0];
            }
            if (point[2].y < point[3].y)
            {
                armor.corner[1] = point[2];
                armor.corner[2] = point[3];
            }
            else
            {
                armor.corner[1] = point[3];
                armor.corner[2] = point[2];
            }
        }

        armor.angle = (lightBar_1.angle + lightBar_2.angle) / 2;
        armor.length = (lightBar_1.length + lightBar_2.length) / 2;
        float x = (lightBar_1.center.x - lightBar_2.center.x) * (lightBar_1.center.x - lightBar_2.center.x);
        float y = (lightBar_1.center.y - lightBar_2.center.y) * (lightBar_1.center.y - lightBar_2.center.y);
        armor.width = sqrt(x + y);
        armor.lengthWidthRatio = armor.width / armor.length;
        armor.area = armor.length * armor.width;
        armor.distance = armor.center.x + armor.center.y;
        armor.parallelism = fabs(lightBar_1.angle - lightBar_2.angle);
        armor.horizontalParallelism = (lightBar_1.center.y - lightBar_2.center.y) / armor.width;

        if (lightBar_1.length > lightBar_2.length)
        {
            armor.lightBarRatio = lightBar_2.length / lightBar_1.length;
        }
        else
        {
            armor.lightBarRatio = lightBar_1.length / lightBar_2.length;
        }
        armor.deltaCenter = sqrt((armor.center.x - last_center.x) * (armor.center.x - last_center.x) + (armor.center.y - last_center.y) * (armor.center.y - last_center.y));

        armor_score->checkArmor(armor); //检查armor的特征是否符合
        if (armor.disable == 0)
        {
            armor_.push(armor);
        }
        return;
    }
}
