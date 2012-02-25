#include "Sprite.h"

bool Sprite::isPointInside( const Vec2f& pt )
{
	return true;
}

void Sprite::draw()
{
	gl::draw(_tex, _pos);
}
