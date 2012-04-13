#include "App.h"
#include "cinder/Perlin.h"
#include "KinectRoutine.h"
#include "Player.h"

void BodyTheatreApp::update()
{
	if (_activeIdx != INVALID_IDX)
	{
		if (!players[_activeIdx].isAlive())
			_activeIdx = INVALID_IDX;
	}
	_routine->update();
}