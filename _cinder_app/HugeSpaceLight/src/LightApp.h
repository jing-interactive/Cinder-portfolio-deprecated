#pragma once

#include "cinder/app/AppBasic.h"
#include "cinder/Timeline.h"

#include "../../../_common/MiniConfig.h"

#include "cinder/osc/OscSender.h"
#include "cinder/osc/OscListener.h"

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
    void onOscKinectMessage(const osc::Message* msg);
    void onOscPadMessage(const osc::Message* msg);

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

extern Channel         mIdleChannels[2];

struct AnimSquence
{
    vector<Channel> seqs[2];
};
const size_t kIdleAnimCount = 10;
const size_t kKinectAnimCount = 3;
extern AnimSquence mKinectAnims[kKinectAnimCount];
extern AnimSquence mIdleAnims[kIdleAnimCount];

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
extern Channel         mKinectSurfs[2]; // sum of mKinectBullets

extern int             mCurrentAnim;
extern int             ANIMATION;

extern int             mProbeConfig;
extern struct Config*  mCurrentConfig;
