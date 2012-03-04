#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	Font font;
	const int n_countdown = 2;
	string welcome = toUtf8(L"ºÏÓ°ÁôÄî");
}

void StateTakephoto::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
	if (!font)
		font = Font("STHupo", 64);
}

void StateTakephoto::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_sharepic);
}

void StateTakephoto::draw()
{
	gl::color(1,1,1);
	gl::draw(_app._tex_selected);
	gl::drawStringCentered(welcome, pos, clr, font);
}

void StateTakephoto::exit()
{

}
