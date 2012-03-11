#include "Spark.h"
#include "LedManager.h"
#include <cinder/Rand.h>
#include <cinder/Utilities.h>

using namespace ci;

namespace
{
	const int MIN_LIFE = 2;
	const int MAX_LIFE = 4;
	const float MIN_SPEED = 0.01f;
	const float MAX_SPEED = 0.04f;
	const int radius_x = 2;
	const int radius_y = 2;
	const int radius_z = 4;
}

Spark::Spark()
{
	min.set(0,0,0);
	max.set(LedManager::W,LedManager::H,LedManager::Z);
	reset();
}

void Spark::reset()
{
	int x = randInt(min.x, max.x);
	int y = randInt(min.y, max.y);
	int z = randInt(min.z, max.z);
	idx = LedManager::index(x,y,z);
	life = randInt(MIN_LIFE, MAX_LIFE);
	speed = randFloat(MIN_SPEED, MAX_SPEED);

	speed_accum = 0;
	speed_accum = 0;
}

void Spark::update(int dev_id)
{
	speed_accum += speed;
	float k = sinf(speed_accum);
	if (k < 0)
	{
		if (--life <= 0)
		{
			LedManager::get(dev_id).setLedColor(idx, LedManager::getDarkColor());
			reset();
			return;
		}
		k = -k;
	}
	
	k = lmap<float>(k*k, 0, 1, 0, 250);
	LedManager::get(dev_id).setLedColor(idx,LedManager::getDarkColor((int)k));
}

void Spark::setCenter( const Vec3i& pos )
{
	min.x = constrain(pos.x-radius_x,0,LedManager::W-1);
	max.x = constrain(pos.x+radius_x,0,LedManager::W-1);
	min.y = constrain(pos.y-radius_y,0,LedManager::H-1);
	max.y = constrain(pos.y+radius_y,0,LedManager::H-1);
	min.z = constrain(pos.z-radius_z,0,LedManager::Z-1);
	max.z = constrain(pos.z+radius_z,0,LedManager::Z-1);
}
