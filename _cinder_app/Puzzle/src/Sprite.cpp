#include "Sprite.h"
#include "cinder/Rand.h"
#include "cinder/ImageIo.h"

// #include "opencv2/imgproc/imgproc.hpp"
// 
// using cv::pointPolygonTest;
// 
// #ifdef _DEBUG
// #pragma comment(lib, "opencv_imgproc233d.lib")
// #else
// #pragma comment(lib, "opencv_imgproc233.lib")
// #endif

namespace
{
	const float THRESH_DIST = 70.0f;
	const float k_Rotate = 5.0f;
	const float ABOUT_ZERO = 10;
}

bool Sprite::isPointInside( const Vec2f& pt )
{
	Vec2f polar = toPolar(pt - _center);
	polar.y -= toRadians(_degree);
	Vec2f local = fromPolar(polar);
	if (local.x > -_size.x/2 && local.x < _size.x/2 && 
		local.y > -_size.y/2 && local.y < _size.y/2)
		return true;
	else
		return false;
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
	glLineWidth(6);
	if (_pos_state == CORRECT)
		drawBox(Color8u(0,255,0));
	else if (_pos_state == HALF)
		drawBox(Color8u(255,0,0), 3);
}

void Sprite::drawBox(const Color8u& clr, int k)
{	
	gl::pushModelView();
	{
		gl::translate(_center);
		gl::rotate(_degree);
		gl::scale(_scale, _scale);
		gl::translate(_size*-0.5);
		gl::color(clr);
		gl::drawStrokedRoundedRect(Rectf(k,k,_size.x-k, _size.y-k), 5);
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
	spr->_pos_state = WRONG;
	spr->_deg_ok = true;

	return spr;
}

void Sprite::setPosFromCursor( const Vec2f& pos )
{
	Vec2f new_p = pos - _pivot;
	int x = static_cast<int>(new_p.x);
	int y = static_cast<int>(new_p.y);
	PosState prev_state = _pos_state;
	if (abs(x%_size.x - _size.x*0.5f) +  abs(y%_size.y - _size.y*0.5f) < THRESH_DIST)
	{
		int i = x/_size.x;
		int j = y/_size.y;
		_center.value().set((i+0.5f)*_size.x, (j+0.5f)*_size.y);
		if (i == _idx.x && j == _idx.y)
		{
			_z = -1000;
			_pos_state = CORRECT;
		}
		else
			_pos_state = HALF;
	}
	else
	{
		_pos_state = WRONG;
		_center = new_p;
	}

	if (_pos_state != WRONG && prev_state != _pos_state)
		_z -= 10;
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
	return _pos_state == CORRECT && _deg_ok;
}
