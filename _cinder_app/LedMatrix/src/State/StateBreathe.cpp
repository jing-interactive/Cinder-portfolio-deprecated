#include <cinder/Utilities.h>
#include <cinder/Rand.h>
#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "Config.h"

using namespace ci;

namespace
{
	const int n_items = LedManager::W*LedManager::H;
}

struct BreatheLine
{
	float root_alpha;//the alpha value of the root
	float min_root_alpha, max_root_alpha;
	BreatheLine()
	{
		reset();
	}

	void update(int dev, int line_id, float sin_value, float breath_decay, float thresh_alpha)
	{
		root_alpha = lmap<float>(abs(sin_value), 0, 1, 
			min_root_alpha,max_root_alpha);
		float alpha = root_alpha;
		for (int z=0;z<LedManager::Z;z++)
		{
			alpha -= breath_decay;
			if (alpha < thresh_alpha)
				break;
			int idx = z*LedManager::W*LedManager::H + line_id;
			LedManager::get(dev).setLedLight(idx, (int)alpha);
		}
	}

	void reset() 
	{
		min_root_alpha = randFloat(BREATH_MIN_ROOT_ALPHA_LOW, BREATH_MIN_ROOT_ALPHA_HIGH);
		max_root_alpha = randFloat(BREATH_MAX_ROOT_ALPHA_LOW, BREATH_MAX_ROOT_ALPHA_HIGH);
	}
};

void StateBreathe::enter()
{
	n_countdown = BREATHE_COUNTDOWN;
	printf("%d %s\n", _dev_id, "[idle]Breathe");
	resetTimer();
	items = new BreatheLine[n_items];
	speed = randFloat(BREATHE_MIN_ALPHA_SPEED, BREATHE_MAX_ALPHA_SPEED);
	min_br = randFloat(BRETHE_MIN_THRESH_ALPHA, BRETHE_MAX_THRESH_ALPHA);
	breathe_decay = randFloat(BREATHE_MIN_DECAY, BREATHE_MAX_DECAY);
	sin_counter = 0; 
}

void StateBreathe::update()
{		
	sin_counter += speed;
	float sinv = sinf(sin_counter);
	if (sin_value > 0 && sinv <= 0)//if hitting bottom
	{
		speed = randFloat(BREATHE_MIN_ALPHA_SPEED, BREATHE_MAX_ALPHA_SPEED);
		min_br = randFloat(BRETHE_MIN_THRESH_ALPHA, BRETHE_MAX_THRESH_ALPHA);
		breathe_decay = randFloat(BREATHE_MIN_DECAY, BREATHE_MAX_DECAY);//generate new breathe_decay
		for (int i=0;i<n_items;i++)
		{
			items[i].reset();
		}
		
	}
	sin_value = sinv;

	for (int i=0;i<n_items;i++)
	{
		items[i].update(_dev_id, i, sin_value, breathe_decay, min_br);
	}

	LedState::update();
}

void StateBreathe::draw()
{

}

void StateBreathe::exit()
{
	delete[] items;
}
