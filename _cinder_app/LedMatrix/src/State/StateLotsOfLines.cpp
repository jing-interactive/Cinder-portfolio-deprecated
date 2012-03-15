#include <cinder/Utilities.h>
#include <cinder/Rand.h>
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "LedLine.h"
#include "Config.h"

namespace
{
	const int n_items = int(LedManager::W*LedManager::H*LOTS_K_ITEMS);
	enum{
		T_COME,
		T_FADE,
		T_CHANGE,
		T_GO,
	};
}

void StateLotsOfLines::enter()
{	
	n_countdown = LOTS_COUNTDOWN;
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
		items[i].setMaxSpeed(LOTS_COME_SPEED);
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
			if (elapsed > n_countdown - SEC_FOR_GO - LOTS_SEC_FADEOUT)
				inner_state = T_FADE;
		}break;	
	case T_FADE:
		{
			LedManager::get(_dev_id).fadeOut(LOTS_SEC_FADEOUT);
			inner_state = T_CHANGE;
		}break;	
	case T_CHANGE:
		{
			if (elapsed > n_countdown - SEC_FOR_GO)
			{
				LedManager::get(_dev_id).fadeIn(LOTS_SEC_FADEIN);
				for (int i=0;i<n_items;i++)
				{
					items[i].setMaxSpeed(LOTS_GO_SPEED);
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
