#include "BodyTheatreApp.h"
#include "cinder/Perlin.h"

#define BOX2D_UPDATE_FPS 60.0f

void BodyTheatreApp::update()
{	
	const int32 velocityIterations = 6;
	const int32 positionIterations = 2;
	_world->Step(1/BOX2D_UPDATE_FPS, velocityIterations, positionIterations);
}