#include <cinder/Utilities.h>
#include <cinder/Rand.h>
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "LedLine.h"

namespace
{  
	const float transition_time = 2;
	const int n_items = int(LedManager::W*LedManager::H*0.75f);
	enum{
		T_COME,
		T_FADE,
		T_CHANGE,
		T_GO,
	};
	const float COME_SPEED = 0.6f;
	const float GO_SPEED = 2.4f;
}

void StateLotsOfLines::enter()
{	
	n_countdown = 60;
	inner_state = T_COME;
	printf("%d %s\n", _dev_id, "[idle]LotsOfStars");
	resetTimer();
	items = new Line[n_items];
	for (int i=0;i<n_items;i++)
	{
		Vec3i target(randInt(LedManager::W),
			randInt(LedManager::H),
			LedManager::Z-1);
		items[i].setTarget(target);
		items[i].setMaxSpeed(COME_SPEED);
	}
}

void StateLotsOfLines::update()
{
	//TODO bug
	double elapsed = getElapsedSeconds();
	for (int i=0;i<n_items;i++)
	{
		items[i].update(_dev_id);
	}
	switch (inner_state)
	{
	case T_COME:
		{
			if (elapsed > n_countdown - transition_time)
				inner_state = T_FADE;
		}break;	
	case T_FADE:
		{
			LedManager::get(_dev_id).fadeOutIn(2,2);
			inner_state = T_CHANGE;
		}break;	
	case T_CHANGE:
		{
		//	if (elapsed > n_come_countdown+transition_time)
			{
				for (int i=0;i<n_items;i++)
				{
					items[i].setMaxSpeed(GO_SPEED);
					items[i].direction = Line::T_GO;
				}
				inner_state = T_GO;
			}
		}break;
	case T_GO:
		{

		}break;
	}
	LedState::update();
}

void StateLotsOfLines::draw()
{

}

void StateLotsOfLines::exit()
{
	delete[] items;
}
