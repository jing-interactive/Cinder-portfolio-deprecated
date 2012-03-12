#include "States.h"
#include "LedMatrixApp.h"
#include <cinder/Utilities.h>
#include "LedManager.h"
#include "LedLine.h"

namespace
{ 
	const int n_items = 40;//duplicates might exist
}

void StateFollowingLines::enter()
{	
	n_countdown = 60;
	printf("%d %s\n", _dev_id, "FollowingStars");
	resetTimer();
	items = new Line[n_items];
	for (int i=0;i<n_items;i++)
	{
		items[i].setMode(Line::T_BOUNCING);
		items[i].setMaxSpeed(0.4f);
		items[i].bouncing_times = 1;
	}
}

void StateFollowingLines::update()
{	
	Vec3i center;
	bool updated = _app.getNewCenter(center, _dev_id);
	for (int i=0;i<n_items;i++)
	{
		if (updated)
			items[i].setTarget(center);
		items[i].update(_dev_id);
	}
	LedState::update();
}

void StateFollowingLines::draw()
{

}

void StateFollowingLines::exit()
{
	delete[] items;
}
