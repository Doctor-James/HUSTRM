#ifndef __ARMOR_DETECTOR__
#define __ARMOR_DETECTOR__

#include "armorThread.h"
#include "lightBarThread.h"
#include "serialPortReadThread.h"
#include "serialPortWriteThread.h"
#include "cameraThread.h"
namespace ly
{
    class armorDetector
    {
    public:
        armorDetector();
        ~armorDetector();
        void wait();

    private:
        config *config_;
        armorThread *armor_;
        cameraThread *camera_;
        lightBarThread *light_;
        serialPortReadThread *serialPortRead_;
        serialPortWriteThread *serialPortWrite_;
    };
}

#endif //__ARMOR_DETECTOR__
