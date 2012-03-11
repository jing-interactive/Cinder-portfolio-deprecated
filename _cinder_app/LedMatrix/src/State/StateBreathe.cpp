#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include <cinder/Utilities.h>
#include "LedManager.h"

namespace
{
	const int n_countdown = 2;
}

void StateBreathe::enter()
{	
	printf("%d %s\n", _dev_id, "[idle]Breathe");
	resetTimer();
}

void StateBreathe::update()
{
	// if (getElapsedSeconds() > n_countdown)
		// _app.changeToState(_app._state_countdown);
}

void StateBreathe::draw()
{

}

void StateBreathe::exit()
{

}
