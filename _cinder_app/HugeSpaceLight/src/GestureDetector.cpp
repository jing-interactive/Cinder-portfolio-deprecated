#include "cinder/Cinder.h"
#include "cinder/app/App.h"
#include "GestureDetector.h"

using namespace std;
using namespace ci;
using namespace ci::app;

GestureDetector::GestureDetector(const Vec3f& direction)
{
    mDirection = direction;
    mIsTracking = false;
    mStartSeconds = 0;
    mLastMessageSecond = 0;
}

void GestureDetector::update(const Vec3f& refPos, const Vec3f& pos)
{
    mEndPos = pos;
    console() << pos << ", " << refPos << endl;

    //if (getElapsedSeconds() - mLastMessageSecond > 0.5f)
    //{
    //    mIsTracking = false;
    //}

    mLastMessageSecond = getElapsedSeconds();

    if (!mIsTracking)
    {
        if (pos.z > refPos.z) //mDirection.dot(pos - refPos) < 0)
        {
            mIsTracking = true;
            mStartPos = pos;
            mStartSeconds = getElapsedSeconds();
            //console() << "start" << endl;
        }
    }
    else
    {
        if (pos.z > mStartPos.z) //mDirection.dot(pos - mStartPos) <= 0)
        {
            mIsTracking = false;
            //console() << "end" << endl;
        }
    }
}

bool GestureDetector::isDetected(float distance, float* speed)
{
    if (!mIsTracking) return false;

    if (getElapsedSeconds() - mStartSeconds > 2)
    {
        mIsTracking = false;
        return false;
    }

    float dist = mDirection.dot(mEndPos - mStartPos);
    //console() << dist << endl;
    if (dist > distance)
    {
        if (speed) 
        {
            *speed = dist / (getElapsedSeconds() - mStartSeconds + 1);
        }
        mIsTracking = false;

        return true;
    }

    return false;
}