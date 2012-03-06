#include "PuzzleApp.h"
#include "States.h"

void PuzzleApp::setupStates()
{
	_state_idle			= shared_ptr<State<PuzzleApp>>(new StateIdle(*this));
	_state_init			= shared_ptr<State<PuzzleApp>>(new StateInit(*this));
	_state_countdown	= shared_ptr<State<PuzzleApp>>(new StateCountdown(*this));
	_state_shuffle		= shared_ptr<State<PuzzleApp>>(new StateShuffle(*this));
	_state_game			= shared_ptr<State<PuzzleApp>>(new StateGame(*this));
	_state_win			= shared_ptr<State<PuzzleApp>>(new StateWin(*this));
	_state_lose			= shared_ptr<State<PuzzleApp>>(new StateLose(*this));
	_state_takephoto	= shared_ptr<State<PuzzleApp>>(new StateTakephoto(*this));
	_state_sharepic		= shared_ptr<State<PuzzleApp>>(new StateSharepic(*this));

	changeToState(_state_init);
}
