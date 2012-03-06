#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	const int n_countdown = 4;
	string welcome = toUtf8(L"游戏挑战失败，有请下一位挑战者");
}

void StateLose::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
}

void StateLose::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_idle);
}

void StateLose::draw()
{
	gl::color(1,1,1);
	gl::draw(_app._tex_selected);
	gl::drawStringCentered(welcome, pos, clr, _app.fnt_big);
}

void StateLose::exit()
{

}
