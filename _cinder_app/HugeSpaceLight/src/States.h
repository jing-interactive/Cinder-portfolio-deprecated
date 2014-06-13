#pragma once

#include "LightApp.h"

struct StateIdle : public State<LightApp>
{
    Anim<float> mIdleFrameIdx;

    GET_SINGLETON_IMPL(StateIdle);

    void enter(LightApp* host);

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
    static State<LightApp>::Ref getRandomState();

    void enter(LightApp* host);
    void exit(LightApp* host);
};

struct StatePusher : public StateInteractive
{
    GET_SINGLETON_IMPL(StatePusher);
    void enter(LightApp* host);
    void update(LightApp* host);
};

struct StateMover : public StateInteractive
{
    GET_SINGLETON_IMPL(StateMover);
    void enter(LightApp* host);
    void update(LightApp* host);
};

struct StateScaler : public StateInteractive
{
    GET_SINGLETON_IMPL(StateScaler);
    void enter(LightApp* host);
    void update(LightApp* host);
};
