#pragma once

#include <cinder/cinder.h>

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
	std::shared_ptr<State<AppType>> _current_state;
	std::shared_ptr<State<AppType>> _prev_state;
	std::shared_ptr<State<AppType>> _global_state;//which never changes

	virtual void setupStates() = 0;

	virtual void update()
	{
		if (_global_state)
			_global_state->update();
		if (_current_state)
			_current_state->update();
	}

	virtual void draw()	
	{
		if (_global_state)
			_global_state->draw();
		if (_current_state)
			_current_state->draw();
	}

	void changeToState(const std::shared_ptr<State<AppType>>& new_state)
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