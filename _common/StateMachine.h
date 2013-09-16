#pragma once

#include "cinder/cinder.h"

template <typename ObjT>
struct State
{
    typedef std::shared_ptr<State<ObjT>> Ref;

	State(ObjT& obj): mObj(obj){}
    virtual void enter()    {};
	virtual void update()   {};
	virtual void draw()     {};
	virtual void exit()     {};
protected:
	ObjT& mObj;
};

template <typename ObjT>
struct StateMachine
{
    typedef typename State<ObjT>::Ref StateRef;
	StateRef mCurrentState;
	StateRef mPrevState;
	StateRef mGlobalState;

	void updateIt()
	{
		if (mGlobalState)
			mGlobalState->update();
		if (mCurrentState)
			mCurrentState->update();
	}

	void drawIt()
	{
		if (mGlobalState)
			mGlobalState->draw();
		if (mCurrentState)
			mCurrentState->draw();
	}

	void changeToPreviousState()
	{
		changeToState(mPrevState);
	}

	void changeToState(const StateRef& newState)
	{
		if (mCurrentState)
		{
			mPrevState = mCurrentState;
			mCurrentState->exit();
		}
		mCurrentState = newState;
		mCurrentState->enter();
	}
};