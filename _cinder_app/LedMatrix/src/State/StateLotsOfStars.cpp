#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include <cinder/Utilities.h>
#include "LedManager.h"

namespace
{
	const int n_countdown = 2;
}

void StateLotsOfStars::enter()
{
	resetTimer();
}

void StateLotsOfStars::update()
{
	// if (getElapsedSeconds() > n_countdown)
		// _app.changeToState(_app._state_countdown);
}

void StateLotsOfStars::draw()
{
	gl::color(1,1,1);
	// gl::draw(_app._tex_selected);
	// gl::drawStringCentered(welcome, pos, clr, font);
}

void StateLotsOfStars::exit()
{

}
