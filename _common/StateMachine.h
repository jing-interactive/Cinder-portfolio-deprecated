#pragma once

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
    StateRef mGlobalState;

    ObjT*   mHost;

    StateMachine(ObjT *host)
    {
        mHost = host;
    }

    void updateIt()
    {
        if (mGlobalState)
            mGlobalState->update(mHost);
        if (mCurrentState)
            mCurrentState->update(mHost);
    }

    void drawIt()
    {
        if (mGlobalState)
            mGlobalState->draw(mHost);
        if (mCurrentState)
            mCurrentState->draw(mHost);
    }

    void changeToPreviousState()
    {
        changeToState(mHost, mPrevState);
    }

    void changeToState(const StateRef& newState)
    {
        if (mCurrentState)
        {
            mPrevState = mCurrentState;
            mCurrentState->exit(mHost);
        }
        mCurrentState = newState;
        mCurrentState->enter(mHost);
    }
};