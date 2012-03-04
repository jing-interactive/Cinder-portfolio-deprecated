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
		gl::color(255,255,255);
		gl::drawSolidCircle(pos, 20);
	}
	gl::color(clr);
	gl::drawSolidCircle(pos, 15);
}
