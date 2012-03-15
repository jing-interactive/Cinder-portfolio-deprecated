#include "Spark.h"
#include "LedManager.h"
#include <cinder/Rand.h>
#include <cinder/Utilities.h>
#include "Config.h"

using namespace ci;

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
	life = randInt(SPARK_MIN_LIFE, SPARK_MAX_LIFE);
	speed = randFloat(SPARK_MIN_SPEED, SPARK_MAX_SPEED);

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
			LedManager::get(dev_id).setLedDark(idx);
			reset();
			return;
		}
		k = -k;
	}
	
	k = lmap<float>(k*k, 0, 1, 0, 250);
	LedManager::get(dev_id).setLedLight(idx,k);
}

void Spark::setCenter( const Vec3i& pos )
{
	min.x = constrain(pos.x-SPARK_RADIUS_X,0,LedManager::W-1);
	max.x = constrain(pos.x+SPARK_RADIUS_X,0,LedManager::W-1);
	min.y = constrain(pos.y-SPARK_RADIUS_Y,0,LedManager::H-1);
	max.y = constrain(pos.y+SPARK_RADIUS_Y,0,LedManager::H-1);
	min.z = constrain(pos.z-SPARK_RADIUS_Z,0,LedManager::Z-1);
	max.z = constrain(pos.z+SPARK_RADIUS_Z,0,LedManager::Z-1);
}
