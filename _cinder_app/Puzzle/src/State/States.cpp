#include "States.h"
#include "PuzzleApp.h"

double PuzzleState::getElapsedSeconds()
{
	return _app.getElapsedSeconds() - _time;
}

void PuzzleState::resetTimer()
{
	_time = _app.getElapsedSeconds();
}
