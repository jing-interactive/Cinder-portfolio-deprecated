#include <cinder/Utilities.h>
#include <cinder/Rand.h>
#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"

using namespace ci;

namespace
{
	const int n_items = LedManager::W*LedManager::H;
	const float MIN_ALPHA_SPEED = 0.003f;
	const float MAX_ALPHA_SPEED = 0.008f;
	const float MIN_BREATH_DECAY = 5.4f;
	const float MAX_BREATH_DECAY = 8.4f;
}

struct BreatheLine
{
	float root_alpha;//the alpha value of the root
	float min_root_alpha, max_root_alpha;
	BreatheLine()
	{
		reset();
	}

	void update(int dev, int line_id, float sin_value, float breath_decay, float min_br)
	{
		root_alpha = lmap<float>(abs(sin_value), 0, 1, 
			min_root_alpha,max_root_alpha);
		float alpha = root_alpha;
		for (int z=0;z<LedManager::Z;z++)
		{
			alpha -= breath_decay;
			if (alpha < min_br)
				break;
			int idx = z*LedManager::W*LedManager::H + line_id;
			LedManager::get(dev).setLedLight(idx, (int)alpha);
		}
	}

	void reset() 
	{
		min_root_alpha = randFloat(0,10);
		max_root_alpha = randFloat(170,255);
	}
};

void StateBreathe::enter()
{
	n_countdown = 60;
	printf("%d %s\n", _dev_id, "[idle]Breathe");
	resetTimer();
	items = new BreatheLine[n_items];
	min_br = randFloat(1.0f, 10.0f);
	breathe_decay = randFloat(MIN_BREATH_DECAY, MAX_BREATH_DECAY);
	sin_counter = 0;
	speed = MIN_ALPHA_SPEED;
}

void StateBreathe::update()
{		
	sin_counter += speed;
	float sinv = sinf(sin_counter);
	if (sin_value > 0 && sinv <= 0)//if hitting bottom
	{
		speed = randFloat(MIN_ALPHA_SPEED, MAX_ALPHA_SPEED);
		min_br = randFloat(1.0f, 10.0f);
		breathe_decay = randFloat(MIN_BREATH_DECAY, MAX_BREATH_DECAY);//generate new breathe_decay
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
	LedManager::get(_dev_id).fadeOut(2);
	delete[] items;
}
