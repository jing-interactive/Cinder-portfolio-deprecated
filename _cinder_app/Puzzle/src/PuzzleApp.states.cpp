#include "PuzzleApp.h"
#include "States.h"

void PuzzleApp::setupStates()
{
	_state_idle			= shared_ptr<State>(new StateIdle(*this));
	_state_init			= shared_ptr<State>(new StateInit(*this));
	_state_countdown	= shared_ptr<State>(new StateCountdown(*this));
	_state_shuffle		= shared_ptr<State>(new StateShuffle(*this));
	_state_game			= shared_ptr<State>(new StateGame(*this));
	_state_gameover		= shared_ptr<State>(new StateGameover(*this));
	_state_takephoto	= shared_ptr<State>(new StateTakephoto(*this));
	_state_sharepic		= shared_ptr<State>(new StateSharepic(*this));

	changeToState(_state_init);
}

void PuzzleApp::updateStates()
{
	//global state change is updated here
	//local change is updated inside related class 		

	_current_state->update();
}

void PuzzleApp::changeToState( const shared_ptr<State>& new_state )
{
	if (_current_state)
		_current_state->exit();
	_current_state = new_state;
	_current_state->enter();
}
