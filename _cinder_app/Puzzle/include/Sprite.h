#pragma once
#include "cinder/Surface.h"
#include "cinder/gl/Texture.h"
#include "cinder/Timeline.h"

using namespace ci;

struct Sprite
{
	gl::Texture _tex;
	float _scale;
	Anim<float> _degree;
	Anim<Vec2f> _pos;
	Vec2i _idx;
	Vec2i _size;
	int _z;
	bool isPointInside(const Vec2f& pt);
	void draw();
	static Sprite* createTile( const Surface8u& img, int x, int y, int tile_w, int tile_h );
};

