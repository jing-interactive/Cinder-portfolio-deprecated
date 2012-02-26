#include "Sprite.h"
#include "cinder/Rand.h"
#include "cinder/ImageIo.h"

bool Sprite::isPointInside( const Vec2f& pt )
{
	float dist = pt.distance(_pos);
	return dist < _size.y/2;
}

void Sprite::draw()
{
	gl::pushModelView();
	gl::translate(_pos);
	gl::rotate(_degree);
	gl::scale(_scale, _scale);
	gl::translate(_size*-0.5);
	gl::draw(_tex);
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
	spr->_pos = Vec2f((x+0.5)*tile_w, (y+0.5)*tile_h);

	return spr;
}
