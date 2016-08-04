#ifndef CONSTRUCTION_H
#define CONSTRUCTION_H
#include "openni_grabber1.h"
#include "registration1.h"
#include <pcl/surface/gp3.h>

class construction
{
public:
    construction();
    void GreedyProcess(CloudXYZRGBA::Ptr cloud);
};

#endif // CONSTRUCTION_H
