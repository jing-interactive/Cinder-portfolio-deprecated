#include "States.h"
#include "PuzzleApp.h"

namespace
{
	int seconds;
	string info;
	Vec2f pos;
	Color8u clr(0,0,255);
}

void StateCountdown::enter()
{
	seconds = _app.getElapsedSeconds();
	pos.set(_app.getWindowSize()/2);
}

void StateCountdown::update()
{

}

void StateCountdown::draw()
{
	info = _app.getElapsedSeconds() - seconds;
	gl::drawStringCentered(info, pos, clr);
}

void StateCountdown::exit()
{
}
