#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include <cinder/Utilities.h>
#include "LedManager.h"

namespace
{
	const int n_countdown = 2;
}

void StateAnimal::enter()
{
	resetTimer();
	_app.changeToState(LedState::create(_app, _dev_id, T_SPARK));
}

void StateAnimal::update()
{
	// if (getElapsedSeconds() > n_countdown)
		// _app.changeToState(_app._state_countdown);
}

void StateAnimal::draw()
{
	gl::color(1,1,1);
	// gl::draw(_app._tex_selected);
	// gl::drawStringCentered(welcome, pos, clr, font);
}

void StateAnimal::exit()
{

}
