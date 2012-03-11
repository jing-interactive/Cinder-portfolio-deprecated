#include "LedManager.h"
#include <cinder/gl/gl.h>

using namespace ci;

namespace 
{
	const float CUBE_SIZE = 2.0f;//尺寸
	const float KX = 10;//间隔X
	const float KY = 10;//间隔Y
	const float KZ = 5;//间隔Z
};

LedManager LedManager::mgr[2];

void LedManager::_setup()
{	
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
		leds[i].clr = getDarkColor();
}

void LedManager::draw3d()
{
	for (int z=0;z<Z;z++)
	{
		for (int x=0;x<W;x++)
		{
			for (int y=0;y<H;y++)
			{
				int idx = index(x, y, z);
				if (leds[idx].visible)
				{
					//	gl::pushModelView();
					//	gl::translate(x*KX, y*KY, z*KZ);
					gl::color(leds[idx].clr);
					gl::drawCube(Vec3f(x*KX, y*KY, z*KZ),
						Vec3f(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE));
					//	gl::popModelView();
				}
			}
		}
	}
}

void LedManager::draw2d()
{

}

Vec3f LedManager::getWatchPoint()
{
	return Vec3f(W*1.5f*KX,H*1.3f*KY, Z*0.3f*KZ);
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

ci::ColorA8u LedManager::getDarkColor(uint8_t alpha)
{
	return ci::ColorA8u(229,229,229,alpha);
}

ci::ColorA8u LedManager::getLightColor(uint8_t alpha)
{
	return ci::ColorA8u(50, 179, 225,alpha);
}
