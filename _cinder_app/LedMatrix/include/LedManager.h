#pragma once

#include <cinder/Color.h>
#include <cinder/Timeline.h>

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

	ci::Anim<float> k_alpha;//[0,1)

	static LedManager& get(int device_id);

	Led leds[TOTAL];

	void fadeIn(float sec);
	void fadeOut(float sec);
	//make a global alpha fade out and then fade in 
	void fadeOutIn(float fadeOutSec, float fadeInSec);

	static int index(int x, int y, int z)
	{
		return z*(W*H)+y*W+x;
	}

	static int index(int x, int y)
	{
		return y*W+x;
	}

	void setLedDark(int idx, ci::uint8_t alpha=5);
	void setLedLight(int idx, ci::uint8_t alpha=200);
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
