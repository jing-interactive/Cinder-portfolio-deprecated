#include <cinder/Rand.h>
#include <cinder/Color.h>
#include "LedLine.h"
#include "LedManager.h"

using namespace ci;

namespace
{	
	const int MIN_LIFE = 1;
	const int MAX_LIFE = 2;
	const float MIN_SPEED = 0.3f;
	const float MAX_SPEED = 0.5f;
	const int MIN_SNAKE = 5;
	const int MAX_SNAKE = 30;
	const int radius_x = 2;
	const int radius_y = 3;
	const float MIN_ALPHA_DECAY = 0.8f;
	const float MAX_ALPHA_DECAY = 0.95f;
}

void Line::update(int dev)
{
	if (!visible)
		return;

	int n = snake_z.size();
	if (state == T_COME)
	{
		snake_z[0] += speed/2;
		if (snake_z[0] > target.z)
			state = T_GO;
	}
	else
	{
		snake_z[0] -= speed;
		if (snake_z[0] <= -n)
		{
			if (--life <= 0)
			{
				reset();
			}
			state = T_COME;
		}
	}
	for (int i=n-1;i>0;i--)
		snake_z[i] = snake_z[i-1];
	float alpha = 255;
	for (int i=0;i<n;i++)
	{
		int z = static_cast<int>(snake_z[i]);
		if (z >= 0)
		{
			int idx = LedManager::index(x,y,z);
			LedManager::get(dev).setLedColor(idx, LedManager::getLightColor(alpha));
		}
		alpha *= decay;
	}
}

void Line::reset()
{
	x = randInt(target.x-radius_x, target.x+radius_x);
	y = randInt(target.y-radius_y, target.y+radius_y);
	x = constrain(x, 0, LedManager::W-1);
	y = constrain(y, 0, LedManager::H-1);
	speed = randFloat(MIN_SPEED, MAX_SPEED);
	decay = randFloat(MIN_ALPHA_DECAY, MAX_ALPHA_DECAY);
	state = T_COME;
	life = randInt(MIN_LIFE, MAX_LIFE);
}

Line::Line()
{ 	
	x = y = 0;
	snake_z.resize(randInt(MIN_SNAKE, MAX_SNAKE));
	target.set(LedManager::W/2,LedManager::H/2,LedManager::Z/2);
	reset();
}

void Line::setTarget( const ci::Vec3i& pos )
{
	target = pos;
}
