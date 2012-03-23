#pragma once

#include <cinder/Color.h>

struct Spark
{
	Spark();
	int life;//how many sin T to experience
	float speed;
	int idx;//idx of led
	void reset();
	void update(int device_id);//change led color
	void setCenter(const ci::Vec3i& pos);
private: 
	float speed_accum;
	ci::Vec3i min;
	ci::Vec3i max;	
};
