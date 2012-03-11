#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	const int n_countdown = 2;
	string welcome = toUtf8(L"欢迎参加体感拼图");
}

void StateInit::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
	_app.selectRandomImage();
}

void StateInit::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_countdown);
}

void StateInit::draw()
{
	gl::color(1,1,1);
	gl::draw(_app._tex_selected);
	gl::drawStringCentered(welcome, pos, clr, _app.fnt_big);
}

void StateInit::exit()
{

}
