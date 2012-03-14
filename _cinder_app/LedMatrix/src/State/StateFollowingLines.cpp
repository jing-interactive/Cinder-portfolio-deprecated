#include "States.h"
#include "LedMatrixApp.h"
#include <cinder/Utilities.h>
#include "LedManager.h"
#include "LedLine.h"
#include "Config.h"

void StateFollowingLines::enter()
{	
	n_countdown = FOLLOWING_COUNTDOWN;
	printf("%d %s\n", _dev_id, "FollowingStars");
	resetTimer();
	items = new Line[FOLLOWING_N_ITEMS];
	for (int i=0;i<FOLLOWING_N_ITEMS;i++)
	{
		items[i].setMode(Line::T_BOUNCING);
		items[i].setMaxSpeed(FOLLOWING_MAX_SPEED);
		items[i].bouncing_times = 1;
	}
}

void StateFollowingLines::update()
{	
	Vec3i center;
	bool updated = _app.getNewCenter(center, _dev_id);
	for (int i=0;i<FOLLOWING_N_ITEMS;i++)
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
