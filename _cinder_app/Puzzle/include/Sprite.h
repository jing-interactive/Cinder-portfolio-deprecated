#pragma once

#include "cinder/gl/Texture.h"

using namespace ci;

struct Sprite
{
	gl::Texture _tex;
	Vec2f _pos;
	Vec2i _idx;
	Vec2f _size;
	int _z;
	bool isPointInside(const Vec2f& pt);
	void draw();
};

