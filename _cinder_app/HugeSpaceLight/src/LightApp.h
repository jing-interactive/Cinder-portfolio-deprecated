#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/Timeline.h"

#include "../../../_common/MiniConfig.h"

#include "../../../_common/StateMachine.h"

#pragma warning(disable: 4244)

using namespace ci;
using namespace ci::app;
using namespace std;

struct LightApp : public AppBasic, StateMachine<LightApp>
{
    LightApp();

    void prepareSettings(Settings *settings);

    void setup();
    void shutdown();

    void setupOsc();

    void keyUp(KeyEvent event);

    void update();

    void draw();
};

// 
extern int             mElapsedLoopCount;
extern float           mRandomColorIndex;
extern int             mHour;
extern int             mCurrentHour;

extern Anim<float>     mGlobalAlpha; // for fade-in / fade-out
extern float           mLastKinectMsgSeconds;

struct AnimSquence
{
    vector<Channel> seqs[2];
};
enum
{
    kIdleAnimCount = 10,
    kKinectAnimCount = 3,
    kTotalAnimCount = kIdleAnimCount + kKinectAnimCount + 1
};
extern AnimSquence mAnims[kTotalAnimCount];

struct KinectBullet
{
    KinectBullet(float wavingSpeed);

    void get(Channel* globe, Channel* wall);

    bool isFinished() const;

    float length;
    bool mIsFinished;
    AnimSquence* kinectSeq;
    Anim<float> index;
};

extern list<KinectBullet> mKinectBullets;
extern Channel         mFinalChannels[2];
extern bool            mIsInteractive;

extern int             mCurrentAnim;
extern int             ANIMATION;
extern int             mCurrentFrame;

extern int             mProbeConfig;
extern struct Config*  mCurrentConfig;
