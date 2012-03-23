#include <cinder/Rand.h>
#include <cinder/Color.h>
#include "LedLine.h"
#include "LedManager.h"
#include "Config.h"

using namespace ci;

void Line::update(int dev)
{
	if (!visible)
		return;

	int n = snake_z.size();
	if (direction == T_COME)
	{
		if (T_BOUNCING == mode)
		{
			snake_z[0] += speed/2;
			if (snake_z[0] >= target.z)
				direction = T_GO;
		}
		else
		{
			snake_z[0] += speed;
			if (snake_z[0] >= target.z + n-1)
				reset();
		}
	}
	else if (direction == T_GO)
	{
		snake_z[0] -= speed;
		if (snake_z[0] <= -n)
		{
			if (T_BOUNCING == mode)
			{
				if (--bouncing_times <= 0)
				{
					reset();
				}
				direction = T_COME;
			}
			else
			{
				reset();
			}
		}
	}
	else//T_STAY
	{

	}
	for (int i=n-1;i>0;i--)
		snake_z[i] = snake_z[i-1];
	float alpha = 255;
	for (int i=0;i<n;i++)
	{
		int z = static_cast<int>(snake_z[i]);
		if (z >= 0 && z < LedManager::Z)
		{
			int idx = LedManager::index(x,y,z);
			LedManager::get(dev).setLedLight(idx, (int)alpha);
		}
		alpha *= decay;
	}
}

void Line::reset()
{
	x = randInt(target.x-LINE_RADIUS_X, target.x+LINE_RADIUS_X);
	y = randInt(target.y-LINE_RADIUS_Y, target.y+LINE_RADIUS_Y);
	x = constrain(x, 0, LedManager::W-1);
	y = constrain(y, 0, LedManager::H-1);
	speed = randFloat(max_speed*0.5f, max_speed);
	decay = randFloat(LINE_MIN_ALPHA_DECAY, LINE_MAX_ALPHA_DECAY);
	if (T_BOUNCING == mode)
	{
		snake_z[0] = 0;
		direction = T_COME;
	}
	else if (T_BULLETS == mode)
	{
		if (direction == T_COME)
			snake_z[0] = 0;
		else
			snake_z[0] = LedManager::Z-1;
	}
	
	bouncing_times = randInt(LINE_MIN_LIFE, LINE_MAX_LIFE);
}

Line::Line()
{ 	
	mode = T_BULLETS;
	x = y = 0;
	direction = T_COME;
	max_speed = 0.5f;
	snake_z.resize(randInt(LINE_MIN_SNAKE, LINE_MAX_SNAKE));
	target.set(LedManager::W/2,LedManager::H/2,LedManager::Z-1);
	reset();
}

void Line::setTarget( const ci::Vec3i& pos )
{
	target = pos;
}

void Line::setMode( Mode mode )
{
	this->mode = mode;
}

void Line::setMaxSpeed( float speed )
{
	max_speed = speed;
}
