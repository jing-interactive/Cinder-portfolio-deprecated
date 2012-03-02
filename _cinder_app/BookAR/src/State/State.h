#pragma once

#include <boost/shared_ptr.hpp>

template <typename AppType>
struct State
{
	State(AppType& app):_app(app){}
	virtual void enter() = 0;
	virtual void update() = 0;
	virtual void draw() = 0;
	virtual void exit() = 0;
protected:
	AppType& _app;
};

template <typename AppType>
struct StateMachine
{
	boost::shared_ptr<State<AppType>> _current_state;
	boost::shared_ptr<State<AppType>> _prev_state;
	boost::shared_ptr<State<AppType>> _global_state;//which never changes
	
	virtual void setupStates() = 0;

	virtual void update()
	{
		_global_state->update();
		_current_state->update();
	}

	virtual void draw()	
	{
		_global_state->draw();
		_current_state->draw();
	}

	void changeToState(const boost::shared_ptr<State<AppType>>& new_state)
	{
		if (_current_state)
		{
			_prev_state = _current_state;
			_current_state->exit();
		}
		_current_state = new_state;
		_current_state->enter();
	}
};