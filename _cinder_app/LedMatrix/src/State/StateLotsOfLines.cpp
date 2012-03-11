#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include <cinder/Utilities.h>
#include "LedManager.h"
#include "LedLine.h"

namespace
{
	const int n_countdown = 2;
}

void StateLotsOfLines::enter()
{	
	printf("%d %s\n", _dev_id, "[idle]LotsOfStars");
	resetTimer();
	_app.changeToState(LedState::create(_app, _dev_id, T_SPARK));
}

void StateLotsOfLines::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToRandomInteractiveState(_dev_id);
}

void StateLotsOfLines::draw()
{

}

void StateLotsOfLines::exit()
{

}
