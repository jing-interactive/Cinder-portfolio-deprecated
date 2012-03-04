#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	Font font;
	const int n_countdown = 2;
	string welcome = toUtf8(L"ÕÕÆ¬·ÖÏí");
}

void StateSharepic::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
	if (!font)
		font = Font("Times New Roman", 64);
}

void StateSharepic::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_idle);
}

void StateSharepic::draw()
{
	gl::color(1,1,1);
	gl::draw(_app._tex_selected);
	gl::drawStringCentered(welcome, pos, clr, font);
}

void StateSharepic::exit()
{

}
