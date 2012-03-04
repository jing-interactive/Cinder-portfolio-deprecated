#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	const int n_countdown = 4;
	string welcome = toUtf8(L"ÓÎÏ·½áÊø");
}

void StateGameover::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
}

void StateGameover::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_takephoto);
}

void StateGameover::draw()
{
	gl::color(1,1,1);
	gl::draw(_app._tex_selected);
	gl::drawStringCentered(welcome, pos, clr, _app.fnt_big);
}

void StateGameover::exit()
{

}
