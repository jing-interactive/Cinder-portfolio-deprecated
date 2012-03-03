#pragma once
#include "cinder/Surface.h"
#include "cinder/gl/Texture.h"
#include "cinder/Timeline.h"

using namespace ci;

struct Sprite
{
	enum State
	{
		NORMAL,
		CLICK,
	};

	void addDegree(float deg);
	void setPosFromCursor(const Vec2f& pos);
	void setPivotFromCursor(const Vec2f& pos);
	void drawBox();
	bool isOK();

	State _state;
	gl::Texture _tex;
	float _scale;
	Anim<float> _degree;
	Anim<Vec2f> _center;
	Vec2f _pivot;
	Vec2f _orig_center;
	Vec2i _idx;
	Vec2i _size;
	int _z;
	bool isPointInside(const Vec2f& pt);
	void draw();
	static Sprite* createTile( const Surface8u& img, int x, int y, int tile_w, int tile_h );

private:
	bool _pos_ok;
	bool _deg_ok;
};

