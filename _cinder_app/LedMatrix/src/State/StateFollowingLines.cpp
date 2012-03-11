#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include <cinder/Utilities.h>
#include "LedManager.h"
#include "LedLine.h"

namespace
{
	const int n_countdown = 60;
	const int n_lines = 40;//duplicates might exist
}

void StateFollowingLines::enter()
{	
	printf("%d %s\n", _dev_id, "FollowingStars");
	resetTimer();
	lines = new Line[n_lines];
}

void StateFollowingLines::update()
{	
	Vec3i center;
	bool updated = _app.getNewCenter(center, _dev_id);
	for (int i=0;i<n_lines;i++)
	{
		if (updated)
			lines[i].setTarget(center);
		lines[i].update(_dev_id);
	}
	if (getElapsedSeconds() > n_countdown)
		_app.changeToRandomInteractiveState(_dev_id);
}

void StateFollowingLines::draw()
{

}

void StateFollowingLines::exit()
{
	delete[] lines;
}
