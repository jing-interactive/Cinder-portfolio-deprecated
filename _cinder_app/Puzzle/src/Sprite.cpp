#include "Sprite.h"
#include "cinder/Rand.h"
#include "cinder/ImageIo.h"

namespace
{
	const float THRESH_DIST = 20.0f;
	const float k_Rotate = 5.0f;
	const float ABOUT_ZERO = 10;
}

bool Sprite::isPointInside( const Vec2f& pt )
{
	float dist = pt.distance(_center);
	return dist < _size.y/2;
}

void Sprite::draw()
{
	gl::pushModelView();
	{
		gl::translate(_center);
		gl::rotate(_degree);
		gl::scale(_scale, _scale);
		gl::translate(_size*-0.5);
		gl::draw(_tex);
	}
	gl::popModelView();
}

void Sprite::drawBox()
{
	gl::pushModelView();
	{
		gl::translate(_center);
		gl::rotate(_degree);
		gl::scale(_scale, _scale);
		gl::translate(_size*-0.5);
		glLineWidth(4);
		gl::color(255,255,255);
		gl::drawStrokedRoundedRect(Rectf(0,0,_size.x, _size.y), 5);
	}
	gl::popModelView();
}

Sprite* Sprite::createTile( const Surface8u& img, int x, int y, int tile_w, int tile_h )
{
	Sprite* spr = new Sprite;
	Surface8u tile(img.clone(Area(x*tile_w, y*tile_h, (x+1)*tile_w, (y+1)*tile_h)));
	spr->_idx.set(x,y);
	spr->_size.set(tile_w, tile_h);
	spr->_z = 0;
	spr->_tex = tile;
	spr->_scale = 1.0f;
	spr->_degree = 0;
	spr->_orig_center = spr->_center = Vec2f((x+0.5)*tile_w, (y+0.5)*tile_h);
	spr->_pos_ok = false;
	spr->_deg_ok = false;

	return spr;
}

void Sprite::setPosFromCursor( const Vec2f& pos )
{
	Vec2f new_p = pos - _pivot;
	if (new_p.distance(_orig_center) < THRESH_DIST)
	{
		_pos_ok = true;
		_center = _orig_center;
	}
	else
	{
		_pos_ok = false;
		_center = new_p;
	}
}

void Sprite::setPivotFromCursor( const Vec2f& pos )
{
	_pivot = pos - _center;
}

void Sprite::addDegree( float deg )
{
	float new_d =_degree + deg * k_Rotate;
	int int_degree = static_cast<int>(new_d);
	if (abs(int_degree % 360) < ABOUT_ZERO)
	{
		_degree = 0;
		_deg_ok = true;
//		console() << int_degree <<endl;
	}
	else
	{
		_deg_ok = false;
		_degree = new_d;
	}
}

bool Sprite::isOK()
{
	return _pos_ok && _deg_ok;
}
