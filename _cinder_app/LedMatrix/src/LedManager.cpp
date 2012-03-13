#include "LedManager.h"
#include <cinder/app/App.h>
#include <cinder/Rand.h>

using namespace ci;
using namespace ci::app;

namespace 
{
	const float CUBE_SIZE = 1.0f;
	const float KX = 5;//spacing X
	const float KY = 5;//spacing Y
	const float KZ = 5;//spacing Z
	const int SCR_W = 29;
	const int SCR_H = 28;

	const int SP = 5;//texture spacing

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
			(pos.x+1)*32-SP, (pos.y+1)*32-SP);
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
	k_alpha = 1.0f;

	led_mapping = Surface8u(Z, W*H, true, SurfaceChannelOrder::RGBA);
	reset();
	int inv_array_x[2] = {0, W-1};//第一列和最后一列
	int inv_array_y[2][3] =	{
		//奇数层不可见的索引
		{1, 3, 5},
		//偶数层不可见的索引
		{0, 2, 4}
	};

	//设置不可见的项
	for (int z=0;z<Z;z++)
	{   
		int odd_idx = z%2;//奇数（2n)为0，偶数(2n+1)为1
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
	gl::translate(Vec3f(0, device_id*(KY+2)*LedManager::H, 0));
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
					gl::drawCube(Vec3f(x*KX, y*KY, z*KZ),
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
	float alpha = abs(sin(0.2f*absoluteTime));
	gl::color(0.1f,0.2f,0.25f, alpha);
	gl::draw(tex_particle, current_tex_area[device_id], Rectf(0,scr_y0,SCR_W, scr_y0+SCR_H));	
}

Vec3f LedManager::getWatchPoint()
{
	return Vec3f(W*1.5f*KX,H*1.3f*KY, Z*0.5f*KZ);
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
	setLedColor(idx, ColorA8u(229,229,229,k_alpha*alpha));
}

void LedManager::setLedLight( int idx, ci::uint8_t alpha/*=200*/ )
{
	setLedColor(idx, ColorA8u(50, 179, 225,k_alpha*alpha));
}

Tween<float>::Options LedManager::fadeOutIn( float fadeOutSec, float fadeInSec )
{
	timeline().apply(&k_alpha, 1.0f, 0.0f, fadeOutSec, EaseInQuad());
	return timeline().appendTo(&k_alpha, 0.0f, 1.0f, fadeInSec, EaseOutQuad());
}

Tween<float>::Options LedManager::fadeIn( float sec )
{
	return timeline().apply(&k_alpha, 0.0f, 1.0f, sec, EaseInQuad());
}

Tween<float>::Options LedManager::fadeOut( float sec )
{
	return timeline().apply(&k_alpha, 1.0f, 0.0f, sec, EaseInQuad());
}

void LedManager::setTexture( ImageSourceRef img )
{
	tex_particle = gl::Texture(img);
}
