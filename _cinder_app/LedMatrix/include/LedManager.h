#pragma once

#include <cinder/Color.h>

const int W = 8;
const int H = 6;
const int Z = 45;
const int TOTAL = W*H*Z;

static ci::ColorA8u DarkColor(229,229,229,5);

struct Led
{
	Led():visible(true){}
	ci::ColorA clr;
	bool visible;
};

struct LedManager
{
	static LedManager mgr[2];
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

	LedManager();//forbid ctr
	void _setup();
};
