#pragma once

#include "cinder/Vector.h"

class GestureDetector
{
public:
    GestureDetector(){}

    GestureDetector(const ci::Vec3f& direction);

    void update(const ci::Vec3f& refPos, const ci::Vec3f& pos);

    bool isDetected(float distance, float* speed);

private:
    float mStartSeconds;
    ci::Vec3f mDirection;

    ci::Vec3f mStartPos;
    ci::Vec3f mEndPos;

    bool mIsTracking;
};

