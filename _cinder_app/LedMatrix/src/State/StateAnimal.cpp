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
						LedManager::get(dev).setLedLight(idx, alpha);
					}
				}
			}
		}

	}
	void setCenter(const Vec3f& pos)
	{
		target_center = pos;
	}
};

void StateAnimal::enter()
{	
	n_countdown = ANIMAL_COUNTDOWN;
	item = new Animal;
	printf("%d %s\n", _dev_id, "Animal");
	resetTimer();
}

void StateAnimal::update()
{	
	Vec3i center;
	bool updated = _app.getNewCenter(center, _dev_id);

	if (updated)
		item->setCenter(center);
	item->update(_dev_id, getElapsedSeconds()); 

	LedState::update();	
}

void StateAnimal::draw()
{

}

void StateAnimal::exit()
{
	delete item;
}
