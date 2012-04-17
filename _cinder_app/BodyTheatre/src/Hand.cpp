#include "Hand.h"
#include "cinder/gl/gl.h"

using namespace ci;

Hand::Hand(const ci::ColorA& color):clr(color)
{
	state = NORMAL;
}

void Hand::draw()
{
	if (state != NORMAL)
	{
		gl::color(0,0,0, 0.3f);
		gl::drawSolidCircle(pos, 25);
// 		gl::color(0,0,0, 0.8f);
// 		glLineWidth(2);
// 		gl::drawStrokedCircle(pos, 20);
		gl::color(clr);
		gl::drawSolidCircle(pos, 15);
	}
	else
	{
		gl::color(clr);
		gl::drawSolidCircle(pos, 12);
	}
}
