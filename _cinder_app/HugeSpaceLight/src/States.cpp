#include "States.h"
#include "Config.h"
#include "cinder/ip/Fill.h"
#include <boost/foreach.hpp>
#include "arduino.h"

State<LightApp>::Ref sFadeOutNextState;

int             mElapsedLoopCount;
float           mRandomColorIndex;
int             mHour;
int             mCurrentHour;

Anim<float>     mGlobalAlpha; // for fade-in / fade-out
float           mLastKinectMsgSeconds;

void StateIdle::update(LightApp* host)
{
    mCurrentFrame = (int)mIdleFrameIdx;
    const float duration = mAnims[0].seqs[0].size(); // HACK
    if (mCurrentAnim != -1)
    {
        for (int id=0; id<2; id++)
        {
            mFinalChannels[id] = mAnims[mCurrentAnim].seqs[id][mCurrentFrame];
        }
    }

    if (DEBUG_MODE)
    {
        DEBUG_ANIM = constrain(DEBUG_ANIM, 0, AnimConfig::kKinect);
        ANIMATION = DEBUG_ANIM;
        mElapsedLoopCount = 0;

        if (mIdleFrameIdx >= duration - 1)
        {
            mCurrentAnim = -1;  // manually invalidate
        }
    }
    else
    {
        while (mIdleFrameIdx >= duration - 1 || mElapsedLoopCount >= mCurrentConfig->animConfigs[ANIMATION].loopCount)
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
        timeline().apply(&mIdleFrameIdx, 0.0f, duration - 1, duration / 24.0f);
        mCurrentAnim = ANIMATION;
    }
}

void StateIdle::enter( LightApp* host )
{
    timeline().apply(&mGlobalAlpha, 1.0f, 4.0f);
    console() << "StateIdle: " << mCurrentAnim << endl;
}


void StateFadeOut::enter(LightApp* host)
{
    console() << "StateFadeOut: " << mCurrentAnim << endl;
    timeline().apply(&mGlobalAlpha, 0.0f, FADE_OUT_SECONDS);
        //.finishFn(std::bind(&LightApp::changeToState, host, sFadeOutNextState));
}

void StateFadeOut::update(LightApp* host)
{
    if (getElapsedSeconds() - mLastKinectMsgSeconds > 0.5f) // TODO
    {
        host->changeToState(StateIdle::getSingleton());
        return;
    }

    if (mGlobalAlpha <= 0.0f)
    {
        host->changeToState(sFadeOutNextState);
        return;
    }

    StateIdle::getSingleton()->update(host); // HACK
}

void StateInteractive::enter( LightApp* host )
{
    timeline().apply(&mGlobalAlpha, 1.0f, 2.0f);
    console() << "StateInteractive: " << mCurrentAnim << endl;
    mIsInteractive = true;
}

void StateInteractive::exit( LightApp* host )
{
    //mCurrentAnim = -1;
    mGlobalAlpha = 0.0f;
    mIsInteractive = false;

    sendArduinoMsg(0);
}

void StatePusher::enter( LightApp* host )
{
    StateInteractive::enter(host);
    sendArduinoMsg(1);
}

void StatePusher::update(LightApp* host)
{
    int aliveMovieCount = 0;

    for (int id=0; id<2; id++)
    {
        ip::fill(&mFinalChannels[id], (uint8_t)0);
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

            for (int y=0; y<mFinalChannels[id].getHeight(); y++)
            {
                for (int x=0; x<mFinalChannels[id].getWidth(); x++)
                {
                    const uint8_t* pSrcRGB = surfs[id].getData(Vec2i(x, y));
                    uint8_t* pDstRGB = mFinalChannels[id].getData(Vec2i(x, y));
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
            return;
        }
    }
}


extern float mMoverFrame;
extern float mMoverTargetFrame;

void StateMover::enter( LightApp* host )
{
    mMoverFrame = 0;
    mMoverTargetFrame = 0;
    StateInteractive::enter(host);
    sendArduinoMsg(2);
}

void StateMover::update(LightApp* host)
{
    if (getElapsedSeconds() - mLastKinectMsgSeconds > KINECT_OUTOF_SECONDS)
    {
        host->changeToState(StateIdle::getSingleton());
        return;
    }

    mMoverFrame = lerp(mMoverFrame, mMoverTargetFrame, 0.5f);

    for (int id=0; id<2; id++)
    {
        mFinalChannels[id] = mAnims[kTotalAnimCount - 1].seqs[id][int(mMoverFrame)]; // TODO: random anim index
    }
}

void StateScaler::enter( LightApp* host )
{
    StateInteractive::enter(host);
    sendArduinoMsg(3);
}

void StateScaler::update(LightApp* host)
{
    if (getElapsedSeconds() - mLastKinectMsgSeconds > KINECT_OUTOF_SECONDS)
    {
        host->changeToState(StateIdle::getSingleton());
    }
}

State<LightApp>::Ref StateInteractive::getRandomState()
{
    int num = rand() % 3;
    if (num == 0) return StatePusher::getSingleton();
    else if (num == 1) return StateMover::getSingleton();
    else return StateScaler::getSingleton();
}
