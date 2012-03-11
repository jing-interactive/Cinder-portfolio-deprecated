#pragma once

#include <cinder/Color.h>

struct Led
{
	Led():visible(true){}
	ci::ColorA clr;
	bool visible;
};

struct LedManager
{
	enum{
		W = 8,
		H = 6,
		Z = 45,
		TOTAL = W*H*Z,
	};

	static ci::ColorA8u getDarkColor(ci::uint8_t alpha = 5);
	static ci::ColorA8u getLightColor(ci::uint8_t alpha);

	static LedManager& get(int device_id);

	Led leds[TOTAL];

	static int index(int x, int y, int z)
	{
		return z*(W*H)+y*W+x;
	}

	static int index(int x, int y)
	{
		return y*W+x;
	}

	void setLedColor(int idx, const ci::ColorA& clr);

	//clear the led colors
	void reset();

	//render colorful cubes
	void draw3d();
	//render 2d led mappings
	void draw2d();

	static ci::Vec3f getWatchPoint();

private:

	static LedManager mgr[2];
	LedManager();//forbid ctr
	void _setup();
};
