#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	const int n_countdown = 20;
}

void StateCountdown::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
}

void StateCountdown::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_shuffle);
}

void StateCountdown::draw()
{
	gl::color(Color8u::gray(255));
	gl::draw(_app._tex_selected, _app.getWindowBounds());
	gl::drawStringCentered(toString(n_countdown-(int)getElapsedSeconds()), pos, clr, _app.fnt_big);
}

void StateCountdown::exit()
{
}
