#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

namespace
{
	Vec2f pos;
	Color8u clr(255,255,255);
	Font font;
	const int n_countdown = 2;
	string welcome("You win the game!");
}

void StateGameover::enter()
{
	resetTimer();
	pos.set(_app.getWindowSize()/2);
	if (!font)
		font = Font("Times New Roman", 64);
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
	gl::drawStringCentered(welcome, pos, clr, font);
}

void StateGameover::exit()
{

}
