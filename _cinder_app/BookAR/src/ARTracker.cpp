#include "ARTracker.h"
#include "SDARTracker.h"

ARTracker* ARTracker::create(const std::string& type)
{
    if (type == "SDAR")
        return new SDARTracker;
//     if if (type == "ARToolKitPlus")
//         return new ARToolKitPlusTracker;
    return NULL;
}

