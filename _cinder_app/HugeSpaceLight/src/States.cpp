#include "States.h"
#include "Config.h"
#include "cinder/ip/Fill.h"
#include <boost/foreach.hpp>

State<LightApp>::Ref sFadeOutNextState;

int             mElapsedLoopCount;
float           mRandomColorIndex;
int             mHour;
int             mCurrentHour;

Anim<float>     mGlobalAlpha; // for fade-in / fade-out
float           mLastKinectMsgSeconds;

Channel         mIdleChannels[2];

void StateIdle::update(LightApp* host)
{
    const int duration = mIdleAnims[0].seqs[0].size() - 1;
    if (mCurrentAnim != -1)
    {
        for (int id=0; id<2; id++)
        {
            mIdleChannels[id] = mIdleAnims[mCurrentAnim].seqs[id][mIdleFrameIdx];
        }
    }

    if (DEBUG_MODE)
    {
        DEBUG_ANIM = constrain(DEBUG_ANIM, 0, AnimConfig::kKinect);
        ANIMATION = DEBUG_ANIM;
        mElapsedLoopCount = 0;

        if (mIdleFrameIdx >= duration)
        {
            mCurrentAnim = -1;  // manually invalidate
        }
    }
    else
    {
        while (mIdleFrameIdx >= duration || mElapsedLoopCount >= mCurrentConfig->animConfigs[ANIMATION].loopCount)
        {
            mIdleFrameIdx = 0; // to make the first condition of while loop fail
            if (mElapsedLoopCount < mCurrentConfig->animConfigs[ANIMATION].loopCount)
            {
                mCurrentAnim = -1;  // manually invalidate
                mElapsedLoopCount++;
            }
            else
            {
                ANIMATION = (ANIMATION + 1) % kIdleAnimCount;
                mElapsedLoopCount = 0;
            }
        }
    }

    if (mCurrentAnim != ANIMATION)
    {
        timeline().apply(&mIdleFrameIdx, 0.0f, mIdleAnims[0].seqs[0].size() - 1, 60);
        mCurrentAnim = ANIMATION;
    }
}

void StateIdle::enter( LightApp* host )
{
    mIdleFrameIdx = 0;
    timeline().apply(&mGlobalAlpha, 1.0f, 4.0f);
    console() << "StateIdle: " << mCurrentAnim << endl;
}


void StateFadeOut::enter(LightApp* host)
{
    console() << "StateFadeOut: " << mCurrentAnim << endl;
    timeline().apply(&mGlobalAlpha, MIN_GLOBAL_ALPHA, FADE_OUT_SECONDS);
}

void StateFadeOut::update(LightApp* host)
{
    if (mGlobalAlpha <= MIN_GLOBAL_ALPHA)
    {
        // TODO: finishFn
        host->changeToState(sFadeOutNextState);
    }

    if (getElapsedSeconds() - mLastKinectMsgSeconds > 0.5f) // TODO
    {
        host->changeToState(StateIdle::getSingleton());
    }

    StateIdle::getSingleton()->update(host); // HACK
}

void StateInteractive::update(LightApp* host)
{
    int aliveMovieCount = 0;

    for (int id=0; id<2; id++)
    {
        ip::fill(&mKinectSurfs[id], (uint8_t)0);
    }

    mKinectBullets.remove_if(tr1::mem_fn(&KinectBullet::isFinished));

    BOOST_FOREACH(KinectBullet& anim, mKinectBullets)
    {
        aliveMovieCount++;

        Channel surfs[2];
        anim.get(&surfs[0], &surfs[1]);

        for (int id=0; id<2; id++)
        {
            AnimSquence& kinectSeq = *anim.kinectSeq;

            for (int y=0; y<kinectSeq.seqs[id][0].getHeight(); y++)
            {
                for (int x=0; x<kinectSeq.seqs[id][0].getWidth(); x++)
                {
                    const uint8_t* pSrcRGB = surfs[id].getData(Vec2i(x, y));
                    uint8_t* pDstRGB = mKinectSurfs[id].getData(Vec2i(x, y));
                    pDstRGB[0] = min<int>((int)pDstRGB[0] + pSrcRGB[0], 255);
                }
            }
        }
    }

    if (aliveMovieCount == 0)
    {
        if (getElapsedSeconds() - mLastKinectMsgSeconds > KINECT_OUTOF_SECONDS)
        {
            host->changeToState(StateIdle::getSingleton());
        }
    }
}

void StateInteractive::exit( LightApp* host )
{
    //mCurrentAnim = -1;
    mGlobalAlpha = MIN_GLOBAL_ALPHA;
}

void StateInteractive::enter( LightApp* host )
{
    timeline().apply(&mGlobalAlpha, 1.0f, 2.0f);
    console() << "StateInteractive: " << mCurrentAnim << endl;
}
