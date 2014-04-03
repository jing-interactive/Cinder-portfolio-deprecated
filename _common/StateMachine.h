#pragma once

/* usage

struct CiApp;

struct StateIdle : public State<CiApp>
{
    GET_SINGLETON_IMPL(StateIdle);

    void enter(CiApp* app);
    void update(CiApp* host);
    void draw(CiApp* host);
};

struct StatePlay : public State<CiApp>
{
    GET_SINGLETON_IMPL(StatePlay);

    void enter(CiApp* app);
    void update(CiApp* host);
    void draw(CiApp* host);
};

struct CiApp : public AppBasic, StateMachine<CiApp>
{
    CiApp() : StateMachine<CiApp>(this)
    {

    }
}
*/

template <typename ObjT>
struct State
{
    typedef std::shared_ptr<State<ObjT>> Ref;

    virtual void enter(ObjT* host)    {};
    virtual void update(ObjT* host)   {};
    virtual void draw(ObjT* host)     {};
    virtual void exit(ObjT* host)     {};
};

#define GET_SINGLETON_IMPL(classname) \
    static Ref getSingleton()\
{\
    static Ref sInstance = Ref(new classname);\
    return sInstance;\
}

template <typename ObjT>
struct StateMachine
{
    typedef typename State<ObjT>::Ref StateRef;
    StateRef mCurrentState;
    StateRef mPrevState;

    ObjT*   mHost;

    StateMachine(ObjT *host)
    {
        mHost = host;
    }

    void updateSM()
    {
        if (mCurrentState)
            mCurrentState->update(mHost);
    }

    void drawSM()
    {
        if (mCurrentState)
            mCurrentState->draw(mHost);
    }

    void changeToPreviousState()
    {
        changeToState(mHost, mPrevState);
    }

    void changeToState(const StateRef& newState)
    {
        if (mCurrentState == newState)
            return;

        if (mCurrentState)
        {
            mPrevState = mCurrentState;
            mCurrentState->exit(mHost);
        }
        mCurrentState = newState;
        mCurrentState->enter(mHost);
    }
};
