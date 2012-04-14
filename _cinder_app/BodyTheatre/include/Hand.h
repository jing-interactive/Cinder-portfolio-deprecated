#include "cinder/color.h"

struct Hand
{
	enum State
	{
		NORMAL,
		CLICK,
		DRAG,
	};
	Hand(const ci::ColorA& = ci::ColorA::white());
	void draw();
	ci::Vec2i pos;
	ci::ColorA clr;
	State state;
};