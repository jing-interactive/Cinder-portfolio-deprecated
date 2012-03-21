#include <cinder/Utilities.h>
#include <cinder/Rand.h>
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"
#include "Config.h"

struct Animal
{
	float radius;
	Animal()
	{
		radius = 3;
		sin_value = 0;
		sin_counter = 0;
		speed = randFloat(ANIMAL_MIN_GROW_SPEED, ANIMAL_MAX_GROW_SPEED);
	}
	float sin_counter;
	float sin_value;
	Vec3f center;
	Vec3f target_center;
	float speed;

	void update(int dev, float time)
	{
		sin_counter += speed;
		float tmp = sinf(sin_counter);
		if (sin_value > 0 && tmp <= 0)//if hitting bottom
		{
			speed = randFloat(ANIMAL_MIN_GROW_SPEED, ANIMAL_MAX_GROW_SPEED);
		}
		sin_value = tmp; 
		radius = lmap<float>(abs(sin_value), 0, 1,ANIMAL_MIN_BODY_RADIUS,ANIMAL_MAX_BODY_RADIUS); 
		center = center.lerp(0.1f, target_center);
		Vec3f p;
		for (int x=0;x<LedManager::W;x++)
		{
			p.x = x;
			for (int y=0;y<LedManager::H;y++)
			{
				p.y = y;
				for (int z=0;z<LedManager::Z;z++)
				{
					p.z = z;
					Vec3f diff = p - center;
					bool ok = false;
					if (diff.z >= 1) 
					{
						Vec3f d(diff);
						d.x *= ANIMAL_POS_SCALE_X;
						d.y *= ANIMAL_POS_SCALE_Y;
						d.z *= ANIMAL_POS_SCALE_Z;
						float len = d.length();
						ok = len < radius && len>radius-0.5f;
					}
					else if (diff.z <= -1) 
					{
						Vec3f d(diff);
						d.x *= ANIMAL_NEG_SCALE_X;
						d.y *= ANIMAL_NEG_SCALE_Y;
						d.z *= ANIMAL_NEG_SCALE_Z;
						float len = d.length();
						ok = len < radius/2 && len>radius/2-0.5f;
					}
					if (ok)
					{
						float alpha = lmap<float>(abs(diff.z), 0, 10, 255, 10);
						alpha = constrain<float>(alpha, 0, 255);
						int idx = LedManager::index(x,y,z);
						int beta = LedManager::get(dev).leds[idx].clr.a;
						alpha = constrain<float>(alpha+beta, 0, 255);
						LedManager::get(dev).setLedLight(idx, alpha);
					}
				}
			}
		}

	}
	void setCenter(const Vec3f& pos)
	{
		target_center = pos;
// 		target_center.x = constrain(target_center.x, 1, LedManager::W-1);
// 		target_center.y = constrain(target_center.y, 1, LedManager::H-1);
	}
};

void StateAnimal::enter()
{	
	n_countdown = ANIMAL_COUNTDOWN;
	items = new Animal[2];
	printf("%d %s\n", _dev_id, "Animal");
	resetTimer();
}

void StateAnimal::update()
{	
	vector<Vec3i> centers;
	bool updated = _app.getNewCenter(centers, _dev_id);
	int n_centers = std::min<int>(centers.size(), 2);

	if (updated)
	{ 
		items[0].setCenter(centers[0]);
	}	
	items[0].update(_dev_id, getElapsedSeconds());

	LedState::update();	
}

void StateAnimal::draw()
{

}

void StateAnimal::exit()
{
	delete[] items;
}
