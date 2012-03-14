#include "LedManager.h"
#include <cinder/app/App.h>
#include <cinder/Rand.h>
#include "Config.h"

using namespace ci;
using namespace ci::app;

namespace 
{
	gl::Texture tex_particle;
	
	Area current_tex_area[2];
	Vec2i sub_regions[] = {
		Vec2i(0,0), Vec2i(1,0), Vec2i(3,0), 	
		Vec2i(3,1), 
		Vec2i(3,2), 
		Vec2i(0,3), 
	};
	const int n_sub_regions = _countof(sub_regions);
	void setRandomTexArea(int dev)
	{
		Vec2i pos = sub_regions[randInt(n_sub_regions)];
		current_tex_area[dev].set(pos.x*32+SP, pos.y*32+SP,
			(pos.x+1)*32-SP*2, (pos.y+1)*32-SP*2);
	}

	static struct LedMgrHelper
	{
		LedMgrHelper()
		{
			for (int dev=0;dev<2;dev++)
			{
				LedManager::get(dev).device_id = dev;
				setRandomTexArea(dev);
			}
		}
	}helper;
}

LedManager LedManager::mgr[2]; 

void LedManager::_setup()
{
	LED_MIN_ALPHA = 0.0f;
	LED_MAX_ALPHA = 1.0f;
	k_alpha = LED_MAX_ALPHA;

	led_mapping = Surface8u(Z, W*H, true, SurfaceChannelOrder::RGBA);
	reset();
	int inv_array_x[2] = {0, W-1};//fisrt row && last row
	int inv_array_y[2][3] =	{
		//odd layers
		{1, 3, 5},
		//even layers
		{0, 2, 4}
	};

	//set the visibility
	for (int z=0;z<Z;z++)
	{   
		int odd_idx = z%2;//odd(2n) -> 0, even(2n+1) -> 1
		for (int i =0;i<2;i++)
		{
			int x = inv_array_x[i];
			for (int j=0;j<3;j++)
			{
				int y = inv_array_y[odd_idx][j];
				int idx = index(x, y, z);
				leds[idx].visible = false;
			}
		}
	}
}

void LedManager::reset()
{
	for (int i=0;i<TOTAL;i++)
		setLedDark(i);
}

void LedManager::draw3d()
{
	gl::translate(Vec3f(0, -device_id*(KY+2)*LedManager::H, 0));
	for (int z=0;z<Z;z++)
	{
		for (int x=0;x<W;x++)
		{
			for (int y=0;y<H;y++)
			{
				int idx = index(x, y, z);
				if (leds[idx].visible)
				{
					gl::color(leds[idx].clr);
					gl::drawCube(Vec3f(x*KX, -y*KY, z*KZ),
						Vec3f(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE));
				}
			}
		}
	}
}

void LedManager::draw2d(double absoluteTime)
{
	static int base_offset[]= {0, W*H+SCR_H};
	static int led_offset[]= {0, SCR_H};
	static int scr_offset[]= {W*H, 0};
	int led_y0 = base_offset[device_id] + led_offset[device_id];
	int scr_y0 = base_offset[device_id] + scr_offset[device_id];

	for (int x=0;x<W;x++)
	{
		for (int y=0;y<W;y++)
		{
			for (int z=0;z<Z;z++)
			{
				int idx = index(x,y,z);
				const ColorA8u& clr = leds[idx].clr;
				led_mapping.setPixel(Vec2i(z, x*W+y), clr);
			}
		}
	}
	gl::color(1,1,1);
	gl::pushModelView();
	{
		gl::translate(0,led_y0);
		gl::draw(led_mapping);
	}
	gl::popModelView();
	float alpha = 255*abs(sin(SCR_LED_SPEED*absoluteTime));
	gl::color(ColorA8u(LIGHT_CLR_R, LIGHT_CLR_G, LIGHT_CLR_B,k_alpha*alpha));
	gl::draw(tex_particle, current_tex_area[device_id], Rectf(0,scr_y0,SCR_W, scr_y0+SCR_H));	
}

Vec3f LedManager::getWatchPoint()
{
	return Vec3f(W*3*KX,H*0.5f*KY, Z*1.0f*KZ);
}

void LedManager::setLedColor( int idx, const ColorA& clr )
{
	assert(idx >=0 && idx < TOTAL);
	leds[idx].clr = clr;
}

LedManager::LedManager()
{
	_setup();
}

LedManager& LedManager::get( int device_id )
{
	assert(device_id >= 0 && device_id<2);
	return mgr[device_id];
}

void LedManager::setLedDark( int idx, ci::uint8_t alpha/*=5*/ )
{
	setLedColor(idx, ColorA8u(DARK_CLR_R,DARK_CLR_G,DARK_CLR_B,k_alpha*alpha));
}

void LedManager::setLedLight( int idx, ci::uint8_t alpha/*=200*/ )
{
	setLedColor(idx, ColorA8u(LIGHT_CLR_R, LIGHT_CLR_G, LIGHT_CLR_B,k_alpha*alpha));
}

Tween<float>::Options LedManager::fadeOutIn( float fadeOutSec, float fadeInSec )
{
	timeline().apply(&k_alpha, LED_MAX_ALPHA, LED_MIN_ALPHA, fadeOutSec, EaseInQuad());
	return timeline().appendTo(&k_alpha, LED_MIN_ALPHA, LED_MAX_ALPHA, fadeInSec, EaseOutQuad());
}

Tween<float>::Options LedManager::fadeIn( float sec )
{
	return timeline().apply(&k_alpha, LED_MIN_ALPHA, LED_MAX_ALPHA, sec, EaseInQuad());
}

Tween<float>::Options LedManager::fadeOut( float sec )
{
	return timeline().apply(&k_alpha, LED_MAX_ALPHA, LED_MIN_ALPHA, sec, EaseInQuad());
}

void LedManager::setTexture( ImageSourceRef img )
{
	tex_particle = gl::Texture(img);
}

void LedManager::resetAlpha()
{
	k_alpha = LED_MAX_ALPHA;
}
