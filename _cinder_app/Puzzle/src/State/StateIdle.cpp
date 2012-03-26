#include "States.h"
#include "PuzzleApp.h"
#include "cinder/Utilities.h"

//TODO: make the game active according to body gestures

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
