#include <cinder/Utilities.h>
#include "States.h"
#include "LedMatrixApp.h"
#include "LedManager.h"

namespace
{ 
}

struct Animal
{
	float radius;
	Animal()
	{
		radius = 3;
	}
	Vec3f center;
	Vec3f target_center;
	void update(int dev, float time)
	{
		radius = lmap<float>(abs(sinf(0.5f*time)), 0, 1,2,3.5);
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
						d.x *= 1.5f;
						d.z *= 0.3f;
						float len = d.length();
						ok = len < radius && len>radius-0.5f;
					}
					else if (diff.z <= -1) 
					{
						Vec3f d(diff);
						d.y *= 1.5f;
						d.z *= 0.2f;
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
	n_countdown = 60;
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
