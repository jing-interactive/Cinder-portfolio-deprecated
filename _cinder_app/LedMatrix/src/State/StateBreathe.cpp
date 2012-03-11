#include "LedState.h"
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include <cinder/Utilities.h>
#include <cinder/Rand.h>

using namespace ci;

namespace
{
	const int n_countdown = 60;
	const int n_lines = LedManager::W*LedManager::H;
	const float MIN_ALPHA_SPEED = 0.008f;
	const float MAX_ALPHA_SPEED = 0.06f;
}

struct BreatheLine
{
	float root_alpha;//the alpha value of the root
	float sin_counter;//
	float speed;// 
	float min_root_alpha, max_root_alpha;
	BreatheLine()
	{
		reset();
	}

	void update(int dev, int line_id)
	{
		sin_counter += speed;
		root_alpha = lmap<float>(abs(sinf(sin_counter)), 0, 1, 
			min_root_alpha,max_root_alpha);
		float alpha = root_alpha;
		for (int z=0;z<LedManager::Z;z++)
		{
			alpha -= 5.4f;
			if (alpha < 1.0f)
				break;
			int idx = z*LedManager::W*LedManager::H + line_id;
			LedManager::get(dev).setLedColor(idx, 
				LedManager::getLightColor(alpha));
		}
	}

	void reset() 
	{
		min_root_alpha = randFloat(0,10);
		max_root_alpha = randFloat(200,255);
		sin_counter = 0;
		speed = MIN_ALPHA_SPEED;//randFloat(MIN_ALPHA_SPEED, MAX_ALPHA_SPEED);
	}
};

void StateBreathe::enter()
{
	printf("%d %s\n", _dev_id, "[idle]Breathe");
	resetTimer();
	lines = new BreatheLine[n_lines];
}

void StateBreathe::update()
{
	if (getElapsedSeconds() > n_countdown)
		_app.changeToRandomIdleState(_dev_id);
	for (int i=0;i<n_lines;i++)
	{
		lines[i].update(_dev_id, i);
	}
}

void StateBreathe::draw()
{

}

void StateBreathe::exit()
{
	delete[] lines;
}
