#ifndef UIELEMENT_H
#define UIELEMENT_H

#include "cinder/Area.h"
#include "cinder/gl/Texture.h"
#include "cinder/app/AppBasic.h"

using namespace ci;
using namespace ci::app;

class UIElement
{
public:
	enum State{
		NORMAL,
		OVER,
		CLICK
	};
	UIElement(int x, int y, int width, int height, gl::Texture tex);
	bool isPointIn( const Vec2f &pt );
	bool mouseMove( MouseEvent event );
	bool mouseDown( MouseEvent event );
	void draw();
private:
	State		_state;
	Vec2f		_pos;
	gl::Texture _tex;
	Area		_area;
};

#endif //UIELEMENT_H