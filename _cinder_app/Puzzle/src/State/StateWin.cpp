#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

string timerString;

void setTimeString( const string& str )
{
	timerString = str;
}

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	const int n_countdown = 4;
	string welcome = toUtf8(L"游戏成功，耗时 ");
}

void StateWin::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
}

void StateWin::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_takephoto);
}

void StateWin::draw()
{
	gl::color(1,1,1);
	gl::draw(_app._tex_selected, _app.getWindowBounds()); 
	gl::drawStringCentered(welcome+timerString, pos, clr, _app.fnt_big);
}

void StateWin::exit()
{

}
