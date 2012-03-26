#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"
#include "Hand.h"

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	const int n_countdown = 2;
	string welcome = toUtf8(L"欢迎参加体感拼图\n伸出双手进入游戏");
}

void StateInit::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
	_app.selectRandomImage();
}

void StateInit::update()
{
	if (_app._hands[LEFT]->state == Hand::NORMAL || _app._hands[RIGHT]->state == Hand::NORMAL)//entering gesture
	{
		resetTimer();
	}
	if (getElapsedSeconds() > n_countdown)
		_app.changeToState(_app._state_countdown);
}

void StateInit::draw()
{
	gl::color(1,1,1);
	if (_app._tex_company_intro)
		gl::draw(_app._tex_company_intro, _app.getWindowBounds());
	gl::drawStringCentered(welcome, pos, clr, _app.fnt_big);
}

void StateInit::exit()
{

}
