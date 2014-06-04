#pragma once

#include "LightApp.h"

struct StateIdle : public State<LightApp>
{
    Anim<float> mIdleFrameIdx;

    GET_SINGLETON_IMPL(StateIdle);

    void enter(LightApp* host)
    {
        timeline().apply(&mGlobalAlpha, 1.0f, 4.0f);
        console() << "StateIdle: " << mCurrentAnim << endl;
    }

    void update(LightApp* host);
};

extern State<LightApp>::Ref sFadeOutNextState;

struct StateFadeOut : public State<LightApp>
{
    GET_SINGLETON_IMPL(StateFadeOut);

    void enter(LightApp* host);

    void update(LightApp* host);
};

struct StateInteractive : public State<LightApp>
{
    GET_SINGLETON_IMPL(StateInteractive);

    void enter(LightApp* host)
    {
        timeline().apply(&mGlobalAlpha, 1.0f, 2.0f);
        console() << "StateInteractive: " << mCurrentAnim << endl;
    }

    void update(LightApp* host);

    void exit(LightApp* host)
    {
        //mCurrentAnim = -1;
        mGlobalAlpha = MIN_GLOBAL_ALPHA;
    }
};

