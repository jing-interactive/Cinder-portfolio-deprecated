#pragma once

#include <cinder/Timeline.h>
#include <cinder/gl/Texture.h>
#include <cinder/Utilities.h>

struct Led
{
	Led():visible(true){}
	ci::ColorA8u clr;
	bool visible;
};

struct LedManager
{
	enum{
		W = 6,
		H = 6,
		Z = 45,
		TOTAL = W*H*Z,
	};

	ci::Anim<float> k_alpha;//[k_min_alpha,k_max_alpha)

	void resetAlpha();

	static LedManager& get(int dev);
	static ci::Vec3f getWatchPoint();
	static void setTexture( ci::ImageSourceRef img );

	int device_id;
	ci::Surface8u led_mapping;
	ci::gl::Texture gl_mapping;

	Led leds[TOTAL];

	ci::Tween<float>::Options fadeIn(float sec);
	ci::Tween<float>::Options fadeOut(float sec);
	//make a global alpha fade out and then fade in 
	ci::Tween<float>::Options fadeOutIn(float fadeOutSec, float fadeInSec);

	static int index(int x, int y, int z)
	{
		x = ci::constrain(x, 0, W-1);
		y = ci::constrain(y, 0, H-1);
		z = ci::constrain(z, 0, Z-1);
		return z*(W*H)+y*W+x;
	}

	static int index(int x, int y)
	{
		return y*W+x;
	}

	void setLedDark(int idx, ci::uint8_t alpha=0);
	void setLedLight(int idx, ci::uint8_t alpha=200);
	void setLedColor(int idx, const ci::ColorA& clr);

	//clear the led colors
	void reset();

	//render colorful cubes
	void draw3d();
	//render 2d led mappings
	void draw2d(double absoluteTime, bool scrVisible = true, bool mappingPreCalculated = false);

private:
	static LedManager mgr[2];
	LedManager();//forbid ctr
	void _setup();
};
