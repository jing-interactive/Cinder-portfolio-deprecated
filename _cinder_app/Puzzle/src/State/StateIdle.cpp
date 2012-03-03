#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

void StateIdle::enter()
{
	_app.changeToState(_app._state_init);
}

void StateIdle::update()
{

}

void StateIdle::draw()
{

}

void StateIdle::exit()
{

}
